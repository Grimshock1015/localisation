/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
   
 This file provides the headers for the EV3 localization starter code. Please have a look to
see what is provided by the starter. The corresponding .c file contains the implementation
as well as clearly defined *** TO DO *** sections that correspond to the parts of the 
code that you have to complete in order to implement the localization algorithms.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 What you need to understand thoroughly in order to complete this project:
 
 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly
   
 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.
   
 * The robot control API you will use to get your robot to move, and to acquire 
   sensor data. Please see the API directory and read through the header files and
   attached documentation
   
 Starter code:
 F. Estrada, 2018 - for CSC C85 
 
*/


#ifndef __localization_header
#define __localization_header

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<malloc.h>
#include "./EV3_RobotControl/btcomm.h"

#ifndef HEXKEY
	#define HEXKEY "00:16:53:55:D9:FC"	// R2D2 EV3 Bluetooth address
#endif

int parse_map(unsigned char *map_img, int rx, int ry);
int robot_localization(int *robot_x, int *robot_y, int *direction);
int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y);
// Run localization/navigation with at most max_relocalizations recovery retries.
// Beliefs and sensor history are reset for every localization attempt.
int run_mission(int target_x, int target_y, int max_relocalizations);
int heading_toward_target(int x, int y, int target_x, int target_y);
typedef enum {
 NAV_STEP_ERROR=-1, NAV_STEP_LOST=0, NAV_STEP_MOVED=1, NAV_STEP_ARRIVED=2
} NavigationStepResult;
// Perform at most one street traversal. Invalid inputs return ERROR unchanged.
// LOST invalidates the pose (-1,-1,-1); the caller must relocalize before retrying.
NavigationStepResult navigate_one_step(int *robot_x, int *robot_y, int *direction,
                                      int target_x, int target_y);
int find_street(void);
int drive_along_street(void);
int scan_intersection(int *tl, int *tr, int *br, int *bl);
int turn_at_intersection(int turn_direction);
int calibrate_sensor(void);
int load_colour_calibration(const char *directory);
int get_sensor_color_robust(void);
const char *get_colour_name(int colour);
void reset_sensor_color_filter(void);
#ifndef COLOUR_SENSOR_PORT
#define COLOUR_SENSOR_PORT PORT_3
#endif
#ifndef GYRO_SENSOR_PORT
#define GYRO_SENSOR_PORT PORT_2
#endif

typedef enum {STOP_ERROR=-1, STOP_TIMEOUT=0, STOP_DISTANCE=1, STOP_COLOUR=2, STOP_ANGLE=3} StopReason;

typedef struct
{
 StopReason reason;
 int colour;     // Map colour ID that ended the move; only meaningful when reason==STOP_COLOUR.
} MoveResult;

// Stop colours are map IDs 1..6 (black, blue, green, yellow, red, white); max_revolutions<=0 or timeout_ms<=0 means "no limit" but at least one is required.
MoveResult move_forward_until(const int *stop_colours, int colour_count, double max_revolutions, int timeout_ms);
MoveResult move_backward_until(const int *stop_colours, int colour_count, double max_revolutions, int timeout_ms);
// turn_direction: 0=right, 1=left. Colours/timeout are optional; without them the pivot ends when the angle is reached.
MoveResult pivot_single_wheel(int angle_degrees, int turn_direction, int reverse, const int *stop_colours, int colour_count, int timeout_ms);
MoveResult turn_in_place(int angle_degrees, int turn_direction, int timeout_ms);
MoveResult turn_in_place_until(int angle_degrees, int turn_direction, const int *stop_colours, int colour_count, int timeout_ms);
int report_move_result(const char *label, MoveResult result);
int beep(void);
unsigned char *readPPMimage(const char *filename, int *rx, int*ry);

double score_intersection_scan(const int map_corners[4], const int observed[4], int heading);
int update_beliefs_from_scan(const int observed[4]);
int update_beliefs_after_turn(int turn_direction);
// After a completed intersection-to-intersection trip: 0=direct yellow,
// 1=red boundary turnaround and return to the starting intersection.
int update_beliefs_after_forward(int boundary_return);


#endif
