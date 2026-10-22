/* Real navigation/mission functions with simulated hardware and localization. */
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MOTOR_B 2
#define MOTOR_C 4
#define ALIGNMENT_AXIS_ADVANCE_REV 0.27
typedef enum {STOP_ERROR=-1,STOP_TIMEOUT=0,STOP_DISTANCE=1,STOP_COLOUR=2,STOP_ANGLE=3} StopReason;
typedef struct {StopReason reason;int colour;} MoveResult;
typedef enum {NAV_STEP_ERROR=-1,NAV_STEP_LOST=0,NAV_STEP_MOVED=1,NAV_STEP_ARRIVED=2} NavigationStepResult;

int sx=3,sy=5;
double beliefs[400][4];
static int next_follow_alignment_colour;
static int intersection_axle_centered;
static int advance_calls,yellow_center_calls,marker_colour,centering_failure,advance_failure;
static int physical_x,physical_y,physical_heading;
static int localization_calls,drive_calls,brake_calls,filter_resets;
static int wrong_pose_attempts,failed_localizations,invalid_localized_pose;
static int failed_turns,failed_half_turns,forced_red,brake_failure_call;
static int stale_colour;

static void reset_sensor_color_filter(void) {filter_resets++;stale_colour=0;}

static int robot_localization(int *x,int *y,int *heading)
{
 localization_calls++;
 assert(*x==-1&&*y==-1&&*heading==-1);
 assert(filter_resets==localization_calls&&stale_colour==0&&next_follow_alignment_colour==1);
 assert(intersection_axle_centered==0);
 for (int i=0;i<sx*sy;i++) for (int h=0;h<4;h++)
  assert(fabs(beliefs[i][h]-1.0/(sx*sy*4))<1e-12);
 // Deliberately leave stale state behind to check the next recovery reset.
 memset(beliefs,0,sizeof(beliefs));beliefs[0][0]=1;
 stale_colour=5;next_follow_alignment_colour=4;
 if (localization_calls<=failed_localizations) return 0;
 *x=physical_x;*y=physical_y;*heading=physical_heading;
 if (localization_calls<=wrong_pose_attempts) *x=1;
 if (invalid_localized_pose) *x=-1;
 intersection_axle_centered=1;
 return 1;
}

static int sample_alignment_colour(void) {return marker_colour;}
static int center_on_alignment_marker(int colour)
{
 assert(colour==4&&!intersection_axle_centered);
 yellow_center_calls++;
 return !centering_failure;
}
static MoveResult move_forward_until(const int *colours,int count,double revolutions,int timeout)
{
 assert(colours==NULL&&count==0&&revolutions==0.27&&timeout==5000);
 assert(!intersection_axle_centered&&yellow_center_calls>advance_calls);
 advance_calls++;
 MoveResult result={advance_failure ? STOP_TIMEOUT : STOP_DISTANCE,0};return result;
}

static int BT_motor_port_stop(int motors,int brake)
{
 assert(motors==(MOTOR_B|MOTOR_C)&&brake==1);
 brake_calls++;
 return brake_calls==brake_failure_call ? -1 : 0;
}

static int turn_at_intersection(int direction)
{
 assert(intersection_axle_centered);
 assert(direction==0||direction==1);
 if (failed_turns>0) {failed_turns--;return 0;}
 physical_heading=(physical_heading+(direction==0 ? 1 : 3))%4;
 return 1;
}

static MoveResult turn_in_place(int degrees,int direction,int timeout)
{
 assert(intersection_axle_centered);
 assert(degrees==180&&direction==0&&timeout>0);
 MoveResult result={STOP_ANGLE,0};
 if (failed_half_turns>0) {failed_half_turns--;result.reason=STOP_TIMEOUT;}
 else physical_heading=(physical_heading+2)%4;
 return result;
}

static int report_move_result(const char *label,MoveResult result)
{(void)label;(void)result;return 0;}

static int drive_along_street(void)
{
 assert(intersection_axle_centered&&next_follow_alignment_colour==1);
 intersection_axle_centered=0;
 drive_calls++;
 const int dx[4]={0,1,0,-1},dy[4]={-1,0,1,0};
 int x=physical_x+dx[physical_heading],y=physical_y+dy[physical_heading];
 if (forced_red||x<0||x>=sx||y<0||y>=sy) return 2;
 physical_x=x;physical_y=y;return 1;
}

#include "mission_under_test.inc"

#define CHECK(condition) do {if (!(condition)) {fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#condition);return 0;}} while (0)

static void reset(void)
{
 sx=3;sy=5;physical_x=2;physical_y=0;physical_heading=1;
 localization_calls=drive_calls=brake_calls=filter_resets=0;
 wrong_pose_attempts=failed_localizations=invalid_localized_pose=0;
 failed_turns=failed_half_turns=forced_red=brake_failure_call=0;
 stale_colour=5;next_follow_alignment_colour=4;
 intersection_axle_centered=0;
 advance_calls=yellow_center_calls=centering_failure=advance_failure=0;marker_colour=4;
 memset(beliefs,0,sizeof(beliefs));beliefs[0][0]=1;
}

static int direct_success(void)
{CHECK(run_mission(2,4,2));CHECK(localization_calls==1&&drive_calls==4&&physical_y==4);return 1;}

static int red_then_relocalize(void)
{
 wrong_pose_attempts=1;
 CHECK(run_mission(2,4,2));
 CHECK(localization_calls==2&&filter_resets==2&&drive_calls==5);
 CHECK(physical_x==2&&physical_y==4&&physical_heading==2);
 return 1;
}

static int repeated_loss_is_bounded(void)
{
 forced_red=1;
 CHECK(!run_mission(2,4,2));
 CHECK(localization_calls==3&&filter_resets==3&&drive_calls==3);
 return 1;
}

static int no_retries(void)
{wrong_pose_attempts=1;CHECK(!run_mission(2,4,0));CHECK(localization_calls==1&&drive_calls==1);return 1;}

static int localization_failure_then_success(void)
{failed_localizations=1;CHECK(run_mission(2,4,2));CHECK(localization_calls==2&&drive_calls==4);return 1;}

static int localization_failures_are_bounded(void)
{failed_localizations=10;CHECK(!run_mission(2,4,2));CHECK(localization_calls==3&&drive_calls==0);return 1;}

static int turn_failure_then_success(void)
{failed_turns=1;CHECK(run_mission(2,4,2));CHECK(localization_calls==2&&physical_y==4);return 1;}

static int half_turn_failure_then_success(void)
{
 physical_heading=0;failed_half_turns=1;
 CHECK(run_mission(2,4,2));CHECK(localization_calls==2&&physical_y==4);return 1;
}

static int invalid_pose_is_not_retried(void)
{invalid_localized_pose=1;CHECK(!run_mission(2,4,2));CHECK(localization_calls==1&&drive_calls==0);return 1;}

static int initial_brake_failure(void)
{brake_failure_call=1;CHECK(!run_mission(2,4,2));CHECK(localization_calls==0&&drive_calls==0);return 1;}

static int lost_pose_brake_failure(void)
{wrong_pose_attempts=1;brake_failure_call=2;CHECK(!run_mission(2,4,2));CHECK(localization_calls==1&&drive_calls==1);return 1;}

static int retry_brake_failure(void)
{failed_localizations=1;brake_failure_call=2;CHECK(!run_mission(2,4,2));CHECK(localization_calls==1&&drive_calls==0);return 1;}

static int already_at_destination(void)
{CHECK(run_mission(2,0,2));CHECK(localization_calls==1&&drive_calls==0);return 1;}

static int invalid_mission_inputs(void)
{
 CHECK(!run_mission(3,4,2));CHECK(!run_mission(-1,0,2));CHECK(!run_mission(0,5,2));
 CHECK(!run_mission(0,0,-1));sx=401;sy=1;CHECK(!run_mission(0,0,2));
 sx=0;CHECK(!run_mission(0,0,2));CHECK(brake_calls==0&&localization_calls==0&&drive_calls==0);
 return 1;
}

static int legacy_navigation_contract(void)
{
 CHECK(go_to_target(2,0,1,2,4)==1);
 CHECK(go_to_target(-1,0,1,2,4)==0);
 physical_x=2;physical_y=0;physical_heading=1;forced_red=1;
 CHECK(go_to_target(2,0,1,2,4)==0);
 return 1;
}

static int all_900_routes(void)
{
 for (int y=0;y<5;y++) for (int x=0;x<3;x++)
  for (int ty=0;ty<5;ty++) for (int tx=0;tx<3;tx++) for (int h=0;h<4;h++)
  {
   physical_x=x;physical_y=y;physical_heading=h;drive_calls=0;
   intersection_axle_centered=1;
   CHECK(go_to_target(x,y,h,tx,ty));
   CHECK(physical_x==tx&&physical_y==ty&&drive_calls==abs(tx-x)+abs(ty-y));
  }
 return 1;
}

static int positions_after_arrival_only(void)
{
 physical_x=0;physical_y=1;physical_heading=0;
 CHECK(run_mission(1,2,0));
 CHECK(drive_calls==2&&advance_calls==1&&yellow_center_calls==1);
 CHECK(physical_x==1&&physical_y==2&&!intersection_axle_centered);
 return 1;
}

static int yellow_required_before_turn(void)
{
 int x=2,y=0,h=1;marker_colour=1;
 CHECK(navigate_one_step(&x,&y,&h,2,4)==NAV_STEP_LOST);
 CHECK(x==-1&&y==-1&&h==-1&&physical_heading==1);
 CHECK(drive_calls==0&&advance_calls==0&&yellow_center_calls==0);
 return 1;
}

static int yellow_center_failure(void)
{
 int x=2,y=0,h=1;centering_failure=1;
 CHECK(navigate_one_step(&x,&y,&h,2,4)==NAV_STEP_LOST);
 CHECK(physical_heading==1&&drive_calls==0&&advance_calls==0);
 return 1;
}

static int axle_advance_failure(void)
{
 int x=2,y=0,h=1;advance_failure=1;
 CHECK(navigate_one_step(&x,&y,&h,2,4)==NAV_STEP_LOST);
 CHECK(physical_heading==1&&drive_calls==0&&advance_calls==1&&!intersection_axle_centered);
 return 1;
}

int main(void)
{
 struct Test {const char *name;int (*run)(void);} tests[]={
  {"direct_success",direct_success},{"red_then_relocalize",red_then_relocalize},
  {"repeated_loss_bounded",repeated_loss_is_bounded},{"zero_retries",no_retries},
  {"localization_failure_then_success",localization_failure_then_success},
  {"localization_failure_bounded",localization_failures_are_bounded},
  {"turn_failure_then_success",turn_failure_then_success},
  {"half_turn_failure_then_success",half_turn_failure_then_success},
  {"invalid_pose_no_retry",invalid_pose_is_not_retried},
  {"initial_brake_failure",initial_brake_failure},{"lost_pose_brake_failure",lost_pose_brake_failure},
  {"retry_brake_failure",retry_brake_failure},{"already_arrived",already_at_destination},
  {"invalid_inputs",invalid_mission_inputs},{"legacy_contract",legacy_navigation_contract},
  {"900_full_routes",all_900_routes},
  {"position_after_arrival_only",positions_after_arrival_only},
  {"yellow_before_turn",yellow_required_before_turn},
  {"yellow_center_failure",yellow_center_failure},
  {"axle_advance_failure",axle_advance_failure}
 };
 int failures=0;
 for (unsigned i=0;i<sizeof(tests)/sizeof(tests[0]);i++)
 {
  reset();int passed=tests[i].run();
  printf("TEST,%s,%s\n",tests[i].name,passed ? "PASS" : "FAIL");failures+=!passed;
 }
 return failures ? 1 : 0;
}
