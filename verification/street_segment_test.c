#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <time.h>
#define MOTOR_B 2
#define MOTOR_C 4
#define SENSOR_TO_AXLE_OFFSET_REV 0.27
#define GYRO_SENSOR_PORT 1
static int street_follow_power=10;
static int gyro_reads,gyro_fail,gyro_drift,start_colour,final_colour;
static int brake_calls,brake_fail,encoder_calls,encoder_fail,follow_result,follow_calls;
static int BT_motor_port_stop(int ports,int brake)
{assert(ports==6&&brake==1);return ++brake_calls==brake_fail ? -1 : 0;}
static int BT_read_gyro(int port,int reset,int *angle,int *rate)
{assert(port==1&&reset==0);*angle=gyro_drift ? gyro_reads*3 : 10;*rate=0;return ++gyro_reads==gyro_fail ? 0 : 1;}
static int usleep(unsigned int microseconds) {(void)microseconds;return 0;}
static int sample_alignment_colour(void) {return follow_calls ? final_colour : start_colour;}
static int read_wheel_degrees(int *right,int *left)
{*right=follow_calls ? 360 : 0;*left=*right;return ++encoder_calls==encoder_fail ? -1 : 0;}
static int follow_black_street(void) {follow_calls++;return follow_result;}
static long elapsed_ms(const struct timespec *start) {(void)start;return 1234;}
static const char *get_colour_name(int colour) {(void)colour;return "fixture";}
#include "street_segment_under_test.inc"
static void reset(void)
{
 gyro_reads=gyro_fail=gyro_drift=brake_calls=brake_fail=encoder_calls=encoder_fail=follow_calls=0;
 start_colour=1;final_colour=4;follow_result=1;
}
int main(void)
{
 reset();assert(run_street_segment_test()==1);assert(follow_calls==1&&brake_calls==2);
 reset();start_colour=4;assert(run_street_segment_test()==0&&follow_calls==0);
 reset();start_colour=0;assert(run_street_segment_test()==0&&follow_calls==0);
 reset();gyro_drift=1;assert(run_street_segment_test()==0&&follow_calls==0);
 reset();gyro_fail=3;assert(run_street_segment_test()==0&&follow_calls==0);
 reset();brake_fail=1;assert(run_street_segment_test()==0&&follow_calls==0);
 reset();encoder_fail=1;assert(run_street_segment_test()==0&&follow_calls==0);
 reset();follow_result=2;final_colour=5;assert(run_street_segment_test()==0&&follow_calls==1&&brake_calls==2);
 reset();follow_result=0;assert(run_street_segment_test()==0&&follow_calls==1&&brake_calls==2);
 reset();final_colour=1;assert(run_street_segment_test()==0&&follow_calls==1);
 reset();brake_fail=2;assert(run_street_segment_test()==0&&follow_calls==1);
 reset();encoder_fail=2;assert(run_street_segment_test()==0&&follow_calls==1);
 puts("12/12 one-segment outcome/preflight cases passed.");
 return 0;
}
