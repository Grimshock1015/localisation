/* Production recovery/centring with deterministic sensor events, no hardware. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#define MOTOR_B 2
#define MOTOR_C 4
#define GYRO_SENSOR_PORT 1
#define ROAD_CENTER_STEP_DEGREES 5
#define ROAD_CENTER_MAX_SWEEP_DEGREES 30
static int heading,turn_calls,brake_calls,brake_failure,reset_calls,gyro_calls;
static int events[32],event_count,event_index,default_colour,previous_colour;
static int confirmed_at_turn,enforce_stop_after_yellow,sweep_result;
static int sample_alignment_colour(void)
{
 int colour=event_index<event_count ? events[event_index] : default_colour;
 event_index++;
 if (enforce_stop_after_yellow&&colour==4&&previous_colour==4&&confirmed_at_turn<0)
  confirmed_at_turn=turn_calls;
 previous_colour=colour;
 return colour;
}
static int center_turn_to_heading(int target)
{
 assert(confirmed_at_turn<0); /* No probe/restore after confirmed yellow. */
 heading=target;turn_calls++;return 1;
}
static int BT_read_gyro(int port,int reset,int *angle,int *rate)
{
 assert(port==1&&(reset==0||reset==1));
 gyro_calls++;
 if (reset) {heading=0;reset_calls++;}
 *angle=heading;*rate=0;return 1;
}
static int BT_motor_port_stop(int ports,int brake)
{assert(ports==6&&brake==1);return ++brake_calls==brake_failure ? -1 : 0;}
static int continuous_recovery_sweep(void) {return sweep_result;}
static const char *get_colour_name(int colour) {(void)colour;return "fixture";}
static int center_on_alignment_marker_or_yellow(int marker_colour,int accept_yellow);
#include "recovery_yellow_under_test.inc"
static void reset(const int *script,int count)
{
 assert(count>=0&&count<=32);
 for (int index=0;index<count;index++) events[index]=script[index];
 event_count=count;event_index=turn_calls=brake_calls=brake_failure=reset_calls=gyro_calls=0;
 heading=0;default_colour=6;previous_colour=-1;confirmed_at_turn=-1;
 enforce_stop_after_yellow=1;sweep_result=1;
}
static void assert_yellow(const int *script,int count,int expected_turns,int expected_heading)
{
 reset(script,count);
 assert(center_on_alignment_marker_or_yellow(1,1)==4);
 assert(turn_calls==expected_turns&&heading==expected_heading&&confirmed_at_turn==turn_calls);
 assert(event_index==count&&reset_calls==0&&brake_calls==1);
}
int main(void)
{
 const int entry[]={4,4};assert_yellow(entry,2,0,0);
 const int acquisition[]={6,4,4};assert_yellow(acquisition,3,1,5);
 const int right_edge[]={1,4,4};assert_yellow(right_edge,3,1,5);
 const int restore[]={1,1,6,6,4,4};assert_yellow(restore,6,4,0);
 const int left_edge[]={1,1,6,6,1,4,4};assert_yellow(left_edge,7,5,-5);
 const int midpoint[]={1,1,6,6,1,1,6,6,4,4};assert_yellow(midpoint,10,8,0);
 const int midpoint_fallback[]={1,1,6,6,1,1,6,6,6,4,4};
 assert_yellow(midpoint_fallback,11,9,0);

 const int transient[]={4,1,6,6,1,6,6,1};reset(transient,8);
 assert(center_on_alignment_marker_or_yellow(1,1)==1&&confirmed_at_turn<0&&reset_calls==0);
 const int unknown[]={4,0};reset(unknown,2);
 assert(center_on_alignment_marker_or_yellow(1,1)==0&&confirmed_at_turn<0);
 const int error[]={4,-1};reset(error,2);
 assert(center_on_alignment_marker_or_yellow(1,1)==0&&turn_calls==0&&event_index==2);
 reset(entry,2);brake_failure=1;
 assert(center_on_alignment_marker_or_yellow(1,1)==0&&turn_calls==0&&event_index==1);
 const int midpoint_error[]={1,1,6,6,1,1,6,6,-1};reset(midpoint_error,9);
 assert(center_on_alignment_marker_or_yellow(1,1)==0&&turn_calls==8&&event_index==9);
 reset(midpoint,10);brake_failure=1;
 assert(center_on_alignment_marker_or_yellow(1,1)==0&&turn_calls==8&&event_index==9);

 /* The normal scan/departure API must not turn yellow into black success. */
 reset(NULL,0);default_colour=4;enforce_stop_after_yellow=0;
 assert(center_on_alignment_marker(1)==0&&turn_calls>0&&brake_calls==0);

 const int recovered_yellow[]={4,4,4};reset(recovered_yellow,3);
 assert(recover_black_heading()==4&&turn_calls==0&&reset_calls==0&&brake_calls==2);
 assert(event_index==3);
 const int final_confirmation_lost[]={4,4,0};reset(final_confirmation_lost,3);
 assert(recover_black_heading()==0&&turn_calls==0&&reset_calls==0&&brake_calls==2);
 const int final_yellow[]={4};reset(final_yellow,1);sweep_result=4;
 assert(recover_black_heading()==4&&reset_calls==0&&brake_calls==1&&event_index==1);
 const int lost_confirmation[]={0};reset(lost_confirmation,1);sweep_result=4;
 assert(recover_black_heading()==0&&reset_calls==0&&turn_calls==0);
 reset(final_yellow,1);sweep_result=4;brake_failure=1;
 assert(recover_black_heading()==0&&reset_calls==0&&event_index==0);
 reset(NULL,0);sweep_result=5;
 assert(recover_black_heading()==5&&reset_calls==0&&turn_calls==0&&event_index==0);
 reset(NULL,0);sweep_result=0;
 assert(recover_black_heading()==0&&reset_calls==0&&turn_calls==0);
 const int black_center[]={1,6,6,1,6,6,1};reset(black_center,7);
 assert(recover_black_heading()==1&&reset_calls==1&&confirmed_at_turn<0);
 puts("22/22 recovery-yellow propagation, confirmation, scope and failure cases passed without Bluetooth.");
 return 0;
}
