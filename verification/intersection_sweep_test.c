#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define MOTOR_B 2
#define MOTOR_C 4
#define GYRO_SENSOR_PORT 1
#define ALIGN_SWEEP_TOTAL_DEGREES 360
#define ALIGN_SWEEP_STEP_DEGREES 10
#define ALIGNMENT_AXIS_ADVANCE_REV 0.27
#define INTERSECTION_SWEEP_STEP_DEGREES 5
#define INTERSECTION_SWEEP_COARSE_STEP_DEGREES 10
#define INTERSECTION_SWEEP_FINE_WINDOW_DEGREES 20
#define INTERSECTION_CROSSING_TOLERANCE_DEGREES 25
#define INTERSECTION_SWEEP_MAX_DEGREES 405
#define INTERSECTION_SWEEP_TIMEOUT_MS 180000
typedef enum {STOP_DISTANCE=1} StopReason;
typedef struct {StopReason reason;int colour;} MoveResult;
static int heading,mode,turn_calls,brake_calls,gyro_calls,sample_calls,reset_calls;
static int turn_failure,gyro_failure,sample_failure,brake_failure,time_calls,final_return;
static int scan_mode,scan_phase,wiggle_calls;
static int intersection_axle_centered;
static int missing_sector_mask;
static int scripted_colours[5],scripted_length,scripted_index;
static long elapsed_ms(const struct timespec *start)
{(void)start;return ++time_calls*(mode==7 ? 60000L : 1000L);}
static int center_turn_to_heading(int target)
{
 turn_calls++;
 if (turn_calls==turn_failure) return 0;
 if (target<heading) final_return=1;
 heading=target;
 return 1;
}
static int BT_read_gyro(int port,int reset,int *angle,int *rate)
{
 assert(port==1);
 if (reset) {heading=0;reset_calls++;}
 *angle=heading;*rate=0;
 return ++gyro_calls==gyro_failure ? 0 : 1;
}
static int BT_motor_port_stop(int ports,int brake)
{assert(ports==6&&brake==1);return ++brake_calls==brake_failure ? -1 : 0;}
static int sample_alignment_colour(void)
{
 if (++sample_calls==sample_failure) return -1;
 if (scan_mode&&scan_phase==0) return 4;
 if (mode==6) return 2; /* Invalid start. */
 if (mode==3) return 1; /* Never leaves black. */
 if (mode==4&&final_return) return 6; /* Final stationary verification fails. */
 if (mode==5&&heading==95) return 0; /* Unknown cannot rearm a crossing. */
 if (mode==2&&heading==45) return 1; /* Off-angle spurious black. */
 if (mode==2&&heading==95) return 6; /* One nonblack flicker within a strip. */
 for (int street=0;street<=4;street++)
 {
  if (mode==1&&street==2) continue; /* Missing 180-degree street. */
  if (abs(heading-street*90)<=10) return 1;
 }
 int quadrant=(heading%360)/90;
 if (missing_sector_mask&(1<<quadrant))
 {
  // No building reading in this sector, but confirmed yellow at its edges
  // still establishes departure from black between the four real streets.
  int offset=heading%90;
  return offset>=25&&offset<=65 ? 0 : 4;
 }
 const int buildings[]={2,3,6,3};
 return buildings[quadrant];
}
static const char *get_colour_name(int colour) {(void)colour;return "fixture";}
static int read_sensor_colour_unfiltered(void)
{
 if (scripted_length)
 {
  assert(scripted_index<scripted_length);
  return scripted_colours[scripted_index++];
 }
 return sample_alignment_colour();
}
static int usleep(unsigned int delay) {(void)delay;return 0;}
static int continuous_recovery_sweep(void) {assert(0);return 0;}
static int center_on_alignment_marker(int colour) {assert(colour==4);return 1;}
static MoveResult move_forward_until(const int *colours,int count,double rev,int timeout)
{
 assert(colours==NULL&&count==0&&rev==0.27&&timeout==5000);
 MoveResult result={STOP_DISTANCE,0};return result;
}
static int report_move_result(const char *label,MoveResult result) {(void)label;(void)result;return 0;}
static int wiggle_to_black(void) {wiggle_calls++;scan_phase=1;return 1;}
static int circular_angle_distance(int first,int second)
{int distance=abs(first-second)%360;return distance>180 ? 360-distance : distance;}
#include "intersection_sweep_under_test.inc"
static void reset(int scenario)
{
 mode=scenario;heading=turn_calls=brake_calls=gyro_calls=sample_calls=reset_calls=0;
 turn_failure=gyro_failure=sample_failure=brake_failure=time_calls=final_return=0;
 scan_mode=scan_phase=wiggle_calls=0;
 intersection_axle_centered=0;
 missing_sector_mask=0;
 scripted_length=scripted_index=0;
}
static int run(void)
{
 int readings[36],final=-1;
 int result=intersection_black_sweep(readings,36,&final);
 assert(brake_calls==1&&heading<=405);
 if (result==0) assert(abs(final-360)<=3&&final_return&&reset_calls==1);
 return result;
}
int main(void)
{
 reset(0);assert(run()==0);
 assert(turn_calls<70); /* Fewer stops than the original 77-stop nominal scan. */
 printf("Nominal scan: %d stopped turns and %d colour reads (previous fixed-step baseline: 77 and 390).\n",
   turn_calls,sample_calls);
 reset(1);assert(run()==-1); /* Missing street must not relabel a later crossing. */
 reset(2);assert(run()==0); /* Repeated black and brief noise count only once. */
 reset(3);assert(run()==-1);
 reset(4);assert(run()==-1);
 reset(5);assert(run()==0);
 reset(6);assert(run()==-1&&turn_calls==0);
 reset(7);assert(run()==-1&&turn_calls==2);
 reset(0);gyro_failure=1;assert(run()==-1&&turn_calls==0);
 reset(0);gyro_failure=2;assert(run()==-1&&turn_calls==1);
 reset(0);sample_failure=2;assert(run()==-1);
 reset(0);turn_failure=1;assert(run()==-1);
 reset(0);brake_failure=1;assert(run()==-1);
 reset(0);int final;assert(intersection_black_sweep(NULL,36,&final)==-1&&turn_calls==0);
 reset(0);int readings[36];assert(intersection_black_sweep(readings,35,&final)==-1&&turn_calls==0);
 reset(0);scan_mode=1;int tl,tr,br,bl;
 assert(scan_intersection(&tl,&tr,&br,&bl)==1);
 assert(intersection_axle_centered==1);
 /* Exactly one initial acquisition; no blind black recovery after the sweep. */
 assert(wiggle_calls==1&&tl==3&&tr==2&&br==3&&bl==6&&reset_calls==3);
 const int missing_masks[]={1,3,7,15};
 for (int test=0;test<4;test++)
 {
  reset(0);scan_mode=1;missing_sector_mask=missing_masks[test];
  assert(scan_intersection(&tl,&tr,&br,&bl)==1&&intersection_axle_centered==1);
  assert(tl==((missing_sector_mask&8) ? -1 : 3));
  assert(tr==((missing_sector_mask&1) ? -1 : 2));
  assert(br==((missing_sector_mask&2) ? -1 : 3));
  assert(bl==((missing_sector_mask&4) ? -1 : 6));
  assert(wiggle_calls==1&&reset_calls==3);
 }
 reset(1);scan_mode=1;
 assert(scan_intersection(&tl,&tr,&br,&bl)==0&&intersection_axle_centered==0);
 assert(tl==-1&&tr==-1&&br==-1&&bl==-1); /* Missing street = unverified heading. */
 reset(4);scan_mode=1;
 assert(scan_intersection(&tl,&tr,&br,&bl)==0&&intersection_axle_centered==0);
 assert(tl==-1&&tr==-1&&br==-1&&bl==-1); /* Failed return to starting black. */
 reset(0);scripted_length=5;
 int unanimous[5]={1,1,1,1,1};
 for (int i=0;i<5;i++) scripted_colours[i]=unanimous[i];
 assert(sample_intersection_colour()==1&&scripted_index==4);
 reset(0);scripted_length=5;
 for (int i=0;i<4;i++) scripted_colours[i]=1;
 scripted_colours[4]=-1;
 assert(sample_colour_agreement_mode(4,0)==-1&&scripted_index==5);
 reset(0);scripted_length=5;
 int delayed[5]={1,2,1,1,1};
 for (int i=0;i<5;i++) scripted_colours[i]=delayed[i];
 assert(sample_intersection_colour()==1&&scripted_index==5);
 reset(0);scripted_length=5;
 int split[5]={1,2,1,2,1};
 for (int i=0;i<5;i++) scripted_colours[i]=split[i];
 assert(sample_intersection_colour()==0&&scripted_index==5);
 reset(0);scripted_length=5;scripted_colours[0]=-1;
 assert(sample_intersection_colour()==-1&&scripted_index==1);
 reset(0);scripted_length=5;
 for (int i=0;i<5;i++) scripted_colours[i]=0;
 assert(sample_intersection_colour()==0&&scripted_index==5);
 puts("28/28 counted-sweep, partial-colour, heading-verification and failure cases passed.");
 return 0;
}
