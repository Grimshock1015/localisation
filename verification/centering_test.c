#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define GYRO_SENSOR_PORT 1
#define ROTATE_BOTH 0
#define ROTATE_RIGHT_WHEEL 2
#define ROTATE_LEFT_WHEEL 1
#define MOTOR_B 2
#define MOTOR_C 4
#define TURN_DEFAULT_TIMEOUT_MS 10000
#define TURN_MIN_POWER 10
#define TURN_MAX_POWER 20
#define TURN_GAIN 0.6
#define TURN_TOLERANCE_DEGREES 3
#define TURN_SETTLE_US 150000
#define MOVE_RAMP_UP_MS 500
#define ROAD_CENTER_STEP_DEGREES 5
#define ROAD_CENTER_MAX_SWEEP_DEGREES 30
typedef enum {STOP_ERROR=-1,STOP_TIMEOUT=0,STOP_COLOUR=2,STOP_ANGLE=3} StopReason;
typedef struct {StopReason reason;int colour;} MoveResult;
static int heading,origin,scenario,motor_calls,brake_calls,gyro_calls,gyro_failure,brake_failure;
static int min_heading,max_heading,origin_samples;
static int allow_reset,reset_calls;
static int first_motion_direction;
static long ticks;
static int movement_setup(const int *colours,int count) {(void)colours;(void)count;return 0;}
static int BT_read_gyro(int port,int reset,int *angle,int *rate)
{
 assert(port==1&&(!reset||allow_reset)); /* Centering must never reset the reference. */
 if (reset) {heading=0;reset_calls++;}
 *angle=heading;*rate=0;
 return ++gyro_calls==gyro_failure ? -1 : 1;
}
static int BT_turn(int left_port,int left,int right_port,int right)
{
 assert(left_port==MOTOR_C&&right_port==MOTOR_B&&left==-right);
 motor_calls++;
 if (!first_motion_direction) first_motion_direction=left>0 ? 1 : -1;
 /* Coarse two-degree physical response models small-turn rounding. */
 heading+=left>0 ? 2 : -2;
 if (heading<min_heading) min_heading=heading;
 if (heading>max_heading) max_heading=heading;
 return 0;
}
static MoveResult stop_with(StopReason reason,int colour)
{MoveResult result={++brake_calls==brake_failure ? STOP_ERROR : reason,colour};return result;}
static int BT_motor_port_stop(int ports,int brake)
{assert(ports==(MOTOR_B|MOTOR_C)&&brake==1);return ++brake_calls==brake_failure ? -1 : 0;}
static int hit_stop_colour(const int *colours,int count) {(void)colours;(void)count;return 0;}
static long elapsed_ms(const struct timespec *start) {(void)start;return ++ticks;}
static int usleep(unsigned int usec) {(void)usec;return 0;}
static const char *get_colour_name(int colour) {(void)colour;return "black";}
static int sample_alignment_colour(void)
{
 int offset=heading-origin;
 if (abs(offset)<=TURN_TOLERANCE_DEGREES)
 {
  origin_samples++;
  if (scenario==3&&origin_samples>1) return 6; /* Lost marker on restore. */
 }
 if (scenario==0) return 1; /* Left edge absent. */
 if (scenario==1||scenario==3) return offset>=-10 ? 1 : 6; /* Right edge absent. */
 if (scenario==4) return offset>=10&&offset<=20 ? 1 : 6; /* Search acquisition. */
 if (scenario==5) return 6; /* Search fails both directions. */
 if (scenario==6) return offset>=-20&&offset<=-10 ? 1 : 6; /* Left fallback. */
 return offset>=-10&&offset<=20 ? 1 : 6;
}
static MoveResult turn_in_place(int degrees,int direction,int timeout)
{
 assert(degrees>0&&(direction==0||direction==1)&&timeout>0);
 if (!first_motion_direction) first_motion_direction=direction==0 ? 1 : -1;
 heading+=direction==0 ? degrees : -degrees;
 if (heading<min_heading) min_heading=heading;
 if (heading>max_heading) max_heading=heading;
 MoveResult result={STOP_ANGLE,0};return result;
}
#include "centering_under_test.inc"
static void reset(int mode)
{
 scenario=mode;heading=origin=123;min_heading=max_heading=origin;
 motor_calls=brake_calls=gyro_calls=gyro_failure=brake_failure=origin_samples=0;ticks=0;
 allow_reset=reset_calls=0;
 first_motion_direction=0;
}
int main(void)
{
 reset(0);assert(center_on_alignment_marker(1)==1);assert(abs(heading-origin)<=3);
 reset(1);assert(center_on_alignment_marker(1)==1);assert(abs(heading-origin)<=3);
 assert(min_heading>=origin-ROAD_CENTER_MAX_SWEEP_DEGREES-3&&
        max_heading<=origin+ROAD_CENTER_MAX_SWEEP_DEGREES+3);
 reset(2);assert(center_on_alignment_marker(1)==1);assert(abs(heading-(origin+5))<=5);
 assert(first_motion_direction==1); /* Even an already-found marker checks right first. */
 reset(3);assert(center_on_alignment_marker(1)==0);assert(abs(heading-origin)<=3);
 reset(4);assert(center_on_alignment_marker(1)==1);assert(heading-origin>=10&&heading-origin<=20);
 assert(first_motion_direction==1&&min_heading>=origin-3);
 reset(5);assert(center_on_alignment_marker(1)==0);assert(abs(heading-origin)<=3);
 assert(first_motion_direction==1&&min_heading<origin&&max_heading>origin);
 reset(6);assert(center_on_alignment_marker(1)==1);
 assert(first_motion_direction==1&&heading-origin>=-20&&heading-origin<=-10);
 reset(4);assert(window_wipe_for_colour(1)==1&&heading==origin+10);
 assert(first_motion_direction==1&&min_heading==origin);
 reset(6);assert(window_wipe_for_colour(1)==1&&heading==origin-10);
 assert(first_motion_direction==1&&max_heading==origin+ROAD_CENTER_MAX_SWEEP_DEGREES);
 reset(5);assert(window_wipe_for_colour(1)==0&&heading==origin);
 assert(first_motion_direction==1&&max_heading==origin+ROAD_CENTER_MAX_SWEEP_DEGREES&&
        min_heading==origin-ROAD_CENTER_MAX_SWEEP_DEGREES);
 reset(1);gyro_failure=1;assert(center_on_alignment_marker(1)==0&&motor_calls==0);
 reset(1);brake_failure=1;assert(center_on_alignment_marker(1)==0);
 reset(1);allow_reset=1;
 assert(rotate_gyro_with_power_limits(ROTATE_BOTH,1,90,NULL,0,10000,10,20).reason==STOP_ANGLE);
 assert(reset_calls==1&&abs(heading-90)<=3);
 reset(1);allow_reset=1;
 assert(rotate_gyro_with_power_limits(ROTATE_BOTH,0,90,NULL,0,10000,10,20).reason==STOP_ANGLE);
 assert(reset_calls==1&&abs(heading+90)<=3);
 reset(1);
 assert(rotate_gyro_with_power_limits(ROTATE_BOTH,1,0,NULL,0,10000,10,20).reason==STOP_ERROR);
 assert(motor_calls==0&&gyro_calls==0);
 reset(1);gyro_failure=4;
 assert(rotate_gyro_to_target(ROTATE_BOTH,90,0,NULL,0,10000,10,20).reason==STOP_ERROR);
 assert(motor_calls>0&&brake_calls>0);
 puts("16/16 right-first search, left fallback, centering and turn-failure cases passed without Bluetooth.");
 return 0;
}
