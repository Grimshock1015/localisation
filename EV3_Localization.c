/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
  
 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot. 
 
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you 
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.
 
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
 * a sequence that allows reliable localization.
 *
 * This source keeps the assignment's map parser and localization interfaces while
 * adding incremental EV3 robot control and sensor handling.
 */

#include "EV3_Localization.h"
#include <time.h>

extern int *socket_id;
static int verify_nxt_colour_sensor(void);
static int align_to_street(void);
static long elapsed_ms(const struct timespec *start);
static int follow_black_street(void);
static int wiggle_to_black(void);
static int center_on_alignment_marker(int marker_colour);
static int window_wipe_for_colour(int target_colour);
static int sample_alignment_colour(void);
static int read_wheel_degrees(int *right,int *left);
static int continuous_black_sweep(int *readings,int sample_count,int *final_angle);
static int continuous_recovery_sweep(void);
static MoveResult turn_in_place_alignment(int angle_degrees,int turn_direction,int timeout_ms);
static int next_follow_alignment_colour=1;

#define SENSOR_TO_AXLE_OFFSET_REV 0.27
#define ALIGNMENT_AXIS_ADVANCE_REV SENSOR_TO_AXLE_OFFSET_REV
#define ALIGN_SWEEP_STEP_DEGREES 10
#define ALIGN_SWEEP_TOTAL_DEGREES 360
#define ALIGN_OPPOSITE_TOLERANCE_DEGREES 35
#define ALIGN_SWEEP_POWER 10
#define ALIGN_SWEEP_TIMEOUT_MS 60000
#define ALIGN_TURN_MIN_POWER 15
#define ALIGN_TURN_MAX_POWER 30
#define RECOVERY_SWEEP_RIGHT_DEGREES 60
#define RECOVERY_SWEEP_LEFT_DEGREES 120
#define ROAD_CENTER_STEP_DEGREES 5
#define ROAD_CENTER_MAX_SWEEP_DEGREES 60
#define FIND_STREET_MAX_REV 1.1
#define FIND_STREET_TIMEOUT_MS 22000
#define ALIGNMENT_TURN_OVERSHOOT_DEGREES 0
#define COLOUR_CLASSIFICATION_MAX_DISTANCE_RATIO 0.85

int map[400][4];
int sx,sy;
double beliefs[400][4];

int main(int argc,char *argv[])
{
 char mapname[1024];
 int dest_x,dest_y,rx,ry;
 unsigned char *map_image;
 memset(&map[0][0],0,sizeof(map));
 sx=0;
 sy=0;

 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y calibration_directory [--read-colours]\n");
  fprintf(stderr,"Use -1 -1 for calibration; --read-colours monitors the NXT sensor without movement.\n");
  return 1;
 }
 snprintf(mapname,sizeof(mapname),"%s",argv[1]);
 dest_x=atoi(argv[2]);
 dest_y=atoi(argv[3]);

 if (dest_x==-1&&dest_y==-1)
 {
  if (BT_open(HEXKEY)!=0) return 1;
  int status=calibrate_sensor();
  BT_close();
  return status==0 ? 0 : 1;
 }
 if (argc<5||load_colour_calibration(argv[4])!=0)
 {
  fprintf(stderr,"Pass a valid NXT calibration directory.\n");
  return 1;
 }

 map_image=readPPMimage(mapname,&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to read map image.\n");
  return 1;
 }
 if (parse_map(map_image,rx,ry)==0)
 {
  fprintf(stderr,"Unable to parse map image.\n");
  free(map_image);
  return 1;
 }
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open EV3 Bluetooth connection.\n");
  free(map_image);
  return 1;
 }
 if (verify_nxt_colour_sensor()!=0)
 {
  BT_close();
  free(map_image);
  return 1;
 }

 if (argc>5&&strcmp(argv[5],"--read-colours")==0)
 {
  printf("Reading NXT colours; press Ctrl+C to stop.\n");
  for (;;)
  {
   if (get_sensor_color_robust()<0)
   {
    BT_close();
    free(map_image);
    return 1;
   }
   usleep(200000);
  }
 }

 for (int index=0;index<sx*sy;index++)
  for (int heading=0;heading<4;heading++)
   beliefs[index][heading]=1.0/(double)(sx*sy*4);

 fprintf(stderr,"All set, ready to go.\n");
 int robot_x=-1,robot_y=-1,direction=-1;
 int status=beep()==0&&robot_localization(&robot_x,&robot_y,&direction);
 if (status) printf("Exploration skeleton ended; location estimate remains unknown.\n");
 beep();
 usleep(150000);
 beep();
 BT_close();
 free(map_image);
 return status ? 0 : 1;
}

static int is_street_marker(MoveResult result)
{
 return result.reason==STOP_COLOUR&&(result.colour==1||result.colour==4);
}

static int circular_angle_distance(int first,int second)
{
 int distance=abs(first-second)%360;
 return distance>180 ? 360-distance : distance;
}

static int nearest_black_interval_center(const int *readings,int sample_count,int current_angle)
{
 int start_index=-1;
 for (int index=0;index<sample_count;index++)
  if (readings[index]!=1)
  {
   start_index=index;
   break;
  }
 if (start_index<0) return -1;

 int interval_centers[ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES];
 int interval_count=0;
 int offset=1;
 while (offset<=sample_count)
 {
  if (readings[(start_index+offset)%sample_count]!=1)
  {
   offset++;
   continue;
  }
  int interval_start=offset;
  while (offset<=sample_count&&readings[(start_index+offset)%sample_count]==1) offset++;
  int interval_end=offset-1;
  int center_index=(start_index+(interval_start+interval_end)/2)%sample_count;
  interval_centers[interval_count++]=center_index*ALIGN_SWEEP_STEP_DEGREES;
 }

 if (interval_count==0) return -1;
 int selected_angle=interval_centers[0];
 int best_isolation=-1;
 int smallest_correction=361;
 for (int first=0;first<interval_count;first++)
 {
  int nearest_other=181;
  for (int second=0;second<interval_count;second++)
  {
   if (first==second) continue;
   int distance=circular_angle_distance(interval_centers[first],interval_centers[second]);
   if (distance<nearest_other) nearest_other=distance;
  }
  int correction=abs(circular_angle_distance(interval_centers[first],current_angle));
  if (nearest_other>best_isolation||
      (nearest_other==best_isolation&&correction<smallest_correction))
  {
   best_isolation=nearest_other;
   smallest_correction=correction;
   selected_angle=interval_centers[first];
  }
 }
 return selected_angle;
}

const char *get_colour_name(int colour)
{
 static const char *const names[]={"unknown","black","blue","green","yellow","red","white"};
 if (colour==-1) return "read error";
 if (colour<0||colour>6) return "unknown";
 return names[colour];
}

int beep(void)
{
 int tones[50][3]={{880,200,25},{-1,-1,-1}};
 // return BT_play_tone_sequence(tones);
 return 0;
}

int turn_at_intersection(int turn_direction)
{
 /*
  * This function is used to have the robot turn either left or right at an intersection (obviously your bot can not just
  * drive forward!).
  *
  * If turn_direction=0, turn right, else if turn_direction=1, turn left.
  *
  * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
  * and on a street it can follow.
  *
  * You can use the return value to indicate success or failure, or to inform your code of the state of the bot
  */
  if (turn_direction!=0&&turn_direction!=1) return 0;
  MoveResult turn=turn_in_place(90,turn_direction,10000);
  if (turn.reason!=STOP_ANGLE)
  {
   report_move_result("intersection turn",turn);
   return 0;
  }
  return 1;
}

  static int align_to_street(void)
  {
   int initial_colour=sample_alignment_colour();
   if (initial_colour!=1&&initial_colour!=4)
   {
    fprintf(stderr,"Cannot align from %s; expected black or yellow.\n",get_colour_name(initial_colour));
    initial_colour=continuous_recovery_sweep();
    if (initial_colour!=1&&initial_colour!=4)
    {
     fprintf(stderr,"Recovery sweep did not reacquire black or yellow.\n");
     return 0;
    }
   }
   MoveResult advance=move_forward_until(NULL,0,ALIGNMENT_AXIS_ADVANCE_REV,5000);
   if (advance.reason!=STOP_DISTANCE)
   {
    report_move_result("align: position axle",advance);
    return 0;
   }

   const int sample_count=ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES;
   int readings[ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES]={0};
   int final_angle;
  if (continuous_black_sweep(readings,sample_count,&final_angle)!=0) return 0;

   // Fill brief unknown gaps inside yellow sectors, then aim for the widest sector midpoint.
   for (int index=0;index<sample_count;index++)
   {
    if (readings[index]!=0) continue;
    int gap=0;
    while (gap<3&&readings[(index+gap)%sample_count]==0) gap++;
    if (gap<=2&&readings[(index+sample_count-1)%sample_count]==4&&
        readings[(index+gap)%sample_count]==4)
     for (int fill=0;fill<gap;fill++) readings[(index+fill)%sample_count]=4;
   }

   for (int index=0;index<sample_count;index++)
   {
    if (readings[index]!=0) continue;
    int gap=0;
    while (gap<3&&readings[(index+gap)%sample_count]==0) gap++;
    if (gap<=2&&readings[(index+sample_count-1)%sample_count]==1&&
        readings[(index+gap)%sample_count]==1)
     for (int fill=0;fill<gap;fill++) readings[(index+fill)%sample_count]=1;
   }

   int target_angle=-1,target_width=0,target_colour=4;
   for (int index=0;index<sample_count;index++)
   {
    int previous=(index+sample_count-1)%sample_count;
    if (readings[index]!=4||readings[previous]==4) continue;
    int width=0;
    while (width<sample_count&&readings[(index+width)%sample_count]==4) width++;
    if (width<2) continue;
    if (width>target_width)
    {
     target_width=width;
     target_angle=((index+(width-1)/2)%sample_count)*ALIGN_SWEEP_STEP_DEGREES;
    }
   }

   if (target_angle<0)
   {
    target_colour=1;
    target_angle=nearest_black_interval_center(readings,sample_count,final_angle);
   }
   if (target_angle<0)
   {
    fprintf(stderr,"Alignment sweep found no yellow sector or black interval.\n");
    return 0;
   }

     int current_angle=final_angle%360;
     int correction=current_angle-target_angle;
     if (correction>180) correction-=360;
     if (correction< -180) correction+=360;
     int turn_direction=correction>=0 ? 1 : 0;
     printf("Alignment target %s at %d degrees; turning %s by %d degrees.\n",
       get_colour_name(target_colour),target_angle,
       turn_direction==1 ? "counterclockwise" : "clockwise",abs(correction));
     if (correction!=0)
   {
      int commanded=abs(correction)+ALIGNMENT_TURN_OVERSHOOT_DEGREES;
      if (commanded>360) commanded=360;
      MoveResult center=turn_in_place_alignment(commanded,turn_direction,30000);
    if (center.reason!=STOP_ANGLE)
    {
     report_move_result("align: turn to target sector",center);
     return 0;
    }
   }
   int verified=sample_alignment_colour();
   if (verified!=target_colour)
   {
    fprintf(stderr,"Alignment target was %s, but final reading is %s.\n",
       get_colour_name(target_colour),get_colour_name(verified));
    int recovered=target_colour==1 ? wiggle_to_black() :
      window_wipe_for_colour(target_colour)==target_colour;
    if (!recovered)
    {
     fprintf(stderr,"Alignment recovery did not reacquire %s.\n",get_colour_name(target_colour));
     return 0;
    }
    printf("Recovery sweep reacquired the %s alignment target.\n",get_colour_name(target_colour));
   }
   int gyro_angle,gyro_rate;
   if (BT_read_gyro(GYRO_SENSOR_PORT,1,&gyro_angle,&gyro_rate)!=1) return 0;
  next_follow_alignment_colour=target_colour;
   printf("Aligned to %s sector center at sweep angle %d degrees; gyro reset.\n",
     get_colour_name(target_colour),target_angle);
   return 1;
  }

  static int turn_at_red_and_search(const int *stop_colours)
  {
    MoveResult turn=turn_in_place(180,0,15000);
   if (turn.reason!=STOP_ANGLE) return 0;
   MoveResult search=move_forward_until(stop_colours,3,FIND_STREET_MAX_REV,FIND_STREET_TIMEOUT_MS);
  if (is_street_marker(search)) return 1;
  if (search.reason!=STOP_DISTANCE&&search.reason!=STOP_TIMEOUT) return 0;
  int recovered=continuous_recovery_sweep();
  return recovered==1||recovered==4;
  }

  int find_street(void)
  {
   const int stop_colours[]={1,4,5};
   MoveResult result=move_forward_until(stop_colours,3,FIND_STREET_MAX_REV,FIND_STREET_TIMEOUT_MS);
   report_move_result("find street",result);
   if (is_street_marker(result)) return 1;
   if (result.reason==STOP_COLOUR&&result.colour==5) return turn_at_red_and_search(stop_colours);
  if (result.reason!=STOP_DISTANCE&&result.reason!=STOP_TIMEOUT) return 0;
   fprintf(stderr,"No road or intersection found within the bounded search.\n");
  int recovered=continuous_recovery_sweep();
  if (recovered==1||recovered==4) return 1;
  if (recovered==5) return turn_at_red_and_search(stop_colours);
  return 0;
  }

  int drive_along_street(void)
  {
   return follow_black_street();
  }

  int scan_intersection(int *tl,int *tr,int *br,int *bl)
  {
   if (tl==NULL||tr==NULL||br==NULL||bl==NULL) return 0;
   *tl=*tr=*br=*bl=-1;
   int initial_colour=sample_alignment_colour();
   if (initial_colour!=4)
   {
    fprintf(stderr,"Intersection scan expected yellow, got %s; starting recovery sweep.\n",
      get_colour_name(initial_colour));
    if (continuous_recovery_sweep()!=4)
    {
     fprintf(stderr,"Recovery sweep did not reacquire yellow.\n");
     return 0;
    }
   }
  if (!center_on_alignment_marker(4))
  {
   fprintf(stderr,"Could not center on the yellow intersection before advancing.\n");
   return 0;
  }
   MoveResult advance=move_forward_until(NULL,0,ALIGNMENT_AXIS_ADVANCE_REV,5000);
   if (advance.reason!=STOP_DISTANCE)
   {
    report_move_result("intersection scan: position axle",advance);
    return 0;
   }
  if (!wiggle_to_black())
  {
   fprintf(stderr,"Could not align with black street before intersection sweep.\n");
   return 0;
  }
   int gyro_angle,gyro_rate;
   if (BT_read_gyro(GYRO_SENSOR_PORT,1,&gyro_angle,&gyro_rate)!=1) return 0;
   const int sample_count=ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES;
   int readings[ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES]={0};
   int final_angle;
  if (continuous_black_sweep(readings,sample_count,&final_angle)!=0) return 0;
   const int headings[]={315,45,135,225};
   const char *labels[]={"TL","TR","BR","BL"};
   int *outputs[]={tl,tr,br,bl};
   int success=1;
   for (int corner=0;corner<4;corner++)
   {
    int counts[7]={0};
    for (int index=0;index<sample_count;index++)
     if (circular_angle_distance(index*ALIGN_SWEEP_STEP_DEGREES,headings[corner])<=25&&
      readings[index]>=1&&readings[index]<=6) counts[readings[index]]++;
    int selected=0,selected_count=0,runner_up=0;
    for (int colour=2;colour<=6;colour++)
    {
     if (colour==4||colour==5) continue;
     if (counts[colour]>selected_count)
     {
      runner_up=selected_count;
      selected=colour;
      selected_count=counts[colour];
     }
     else if (counts[colour]>runner_up) runner_up=counts[colour];
    }
    printf("Intersection %s at %d degrees: %s (%d/sector)\n",labels[corner],
      headings[corner],get_colour_name(selected),selected_count);
    if (selected_count<1||selected_count==runner_up) success=0;
    else *outputs[corner]=selected;
   }
   printf("Intersection sweep complete; gently reacquiring the starting black road.\n");
   if (!wiggle_to_black()) return 0;
   if (BT_read_gyro(GYRO_SENSOR_PORT,1,&gyro_angle,&gyro_rate)!=1) return 0;
   printf("Intersection relative readings: TL=%s TR=%s BL=%s BR=%s; gyro zero reset.\n",
     get_colour_name(*tl),get_colour_name(*tr),get_colour_name(*bl),get_colour_name(*br));
   return success;
  }

static int recover_from_red_boundary(void)
{
 printf("Red boundary: turning around and reacquiring the street.\n");
 MoveResult turn=turn_in_place(180,0,15000);
 if (turn.reason!=STOP_ANGLE)
 {
  report_move_result("boundary turnaround",turn);
  return 0;
 }
 if (!find_street()) return 0;
 if (wiggle_to_black()) return 1;
 fprintf(stderr,"Black not found by recovery wipe; running full street alignment.\n");
 return align_to_street();
}

static int drive_to_next_intersection(int *direction)
{
 for (int attempt=0;attempt<2;attempt++)
 {
  int status=drive_along_street();
  if (status==1) return 1;
  if (status!=2)
  {
   fprintf(stderr,"Street following failed before reaching an intersection.\n");
   return 0;
  }
  if (attempt==1||!recover_from_red_boundary())
  {
   fprintf(stderr,"Could not recover from the red boundary.\n");
   return 0;
  }
  *direction=(*direction+2)%4;
 }
 return 0;
}

static int window_wipe_for_colour(int target_colour)
{
 int colour=sample_alignment_colour();
 if (colour==5)
 {
  int confirmation=sample_alignment_colour();
  if (confirmation!=5) colour=confirmation==1||confirmation==4 ? confirmation : 0;
 }
 if (colour<0) return -1;
 if ((target_colour==0&&(colour==1||colour==4||colour==5))||colour==target_colour) return colour;

 printf("Gentle recovery search: left then right in %d-degree steps, up to %d degrees per side.\n",
   ROAD_CENTER_STEP_DEGREES,ROAD_CENTER_MAX_SWEEP_DEGREES);
 for (int direction=1;direction>=0;direction--)
 {
  int turned=0;
  while (turned<ROAD_CENTER_MAX_SWEEP_DEGREES)
  {
   MoveResult turn=turn_in_place(ROAD_CENTER_STEP_DEGREES,direction,10000);
   if (turn.reason!=STOP_ANGLE) return -1;
   turned+=ROAD_CENTER_STEP_DEGREES;
   colour=sample_alignment_colour();
   if (colour==5)
   {
    int confirmation=sample_alignment_colour();
    if (confirmation!=5) colour=confirmation==1||confirmation==4 ? confirmation : 0;
   }
   if (colour<0)
   {
    if (turned>0)
    {
     MoveResult restore=turn_in_place(turned,direction==1 ? 0 : 1,15000);
     if (restore.reason!=STOP_ANGLE) return -1;
    }
    return -1;
   }
   printf("Recovery probe %s %d degrees: %s (%d)\n",
     direction==1 ? "left" : "right",turned,get_colour_name(colour),colour);
   if ((target_colour==0&&(colour==1||colour==4||colour==5))||colour==target_colour)
    return colour;
  }
  if (turned>0)
  {
   MoveResult restore=turn_in_place(turned,direction==1 ? 0 : 1,15000);
   if (restore.reason!=STOP_ANGLE) return -1;
  }
 }
 return 0;
}

static int wiggle_to_black(void)
{
 int result=window_wipe_for_colour(1);
 if (result!=1)
 {
  fprintf(stderr,"Window wipe did not reacquire black road.\n");
  return 0;
 }
 return center_on_alignment_marker(1);
}

int robot_localization(int *robot_x, int *robot_y, int *direction)
{
 /*  This function implements the main robot localization process. You have to write all code that will control the robot
  *  and get it to carry out the actions required to achieve localization.
  *
  *  Localization process:
  *
  *  - Find the street, and drive along the street toward an intersection
  *  - Scan the colours of buildings around the intersection
  *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
  *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
  * 
  *  * We have provided headers for the following functions:
  * 
  *  find_street()
  *  drive_along_street()
  *  scan_intersection()
  *  turn_at_intersection()
  * 
  *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
  *  provided as a suggestion.
  * 
  *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
  *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
  *  other streets than the one your bot was initially placed on.
  * 
  *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
  *  it.
  * 
  *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
  *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
  *  in order to update beliefs.
  * 
  *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
  *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
  * 
  *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
  *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
  * 
  *  The function receives as input pointers to three integer values, these will be used to store the estimated
  *   robot's location and facing direction. The direction is specified as:
  *   0 - UP
  *   1 - RIGHT
  *   2 - BOTTOM
  *   3 - LEFT
  * 
  *  The function's return value is 1 if localization was successful, and 0 otherwise.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 if (robot_x==NULL||robot_y==NULL||direction==NULL) return 0;
 *robot_x=-1;
 *robot_y=-1;
 *direction=0;

 if (!find_street()||!align_to_street()) return 0;
 *direction=0;
 if (!drive_to_next_intersection(direction)) return 0;
 printf("First yellow intersection establishes relative orientation 0.\n");

 for (int intersection=0;intersection<4;intersection++)
 {
  int tl=-1,tr=-1,br=-1,bl=-1;
  if (!scan_intersection(&tl,&tr,&br,&bl))
   fprintf(stderr,"Intersection %d scan failed; readings are unknown, continuing route for now.\n",
     intersection+1);
  printf("Intersection %d relative readings: TL=%s TR=%s BL=%s BR=%s\n",
    intersection+1,get_colour_name(tl),get_colour_name(tr),
    get_colour_name(bl),get_colour_name(br));

  if (intersection==3) break;

  int turn_direction=intersection==0 ? 0 : intersection==1 ? 1 : -1;
  if (turn_direction==0) printf("Route step: turn right at intersection 1.\n");
  else if (turn_direction==1) printf("Route step: turn left at intersection 2.\n");
  else printf("Route step: continue straight at intersection 3.\n");

  if (turn_direction>=0)
  {
   if (!turn_at_intersection(turn_direction)) return 0;
   *direction=(*direction+(turn_direction==0 ? 1 : 3))%4;
  }
  if (!drive_to_next_intersection(direction)) return 0;
 }

 printf("Visited four intersections: right, left, straight; stopping at intersection 4.\n");
 printf("Bayesian localization is not implemented yet.\n");
 return 1;
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y)
{
 /*
  * This function is called once localization has been successful, it performs the actions required to take the robot
  * from its current location to the specified target location. 
  *
  * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
  * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
  * understand how everything works.
  *
  * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
  * should be able to recover.
  * 
  * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
  *          The target's intersection location
  * 
  * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
  */   

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  return(0);  
}

static int calibration_command(const unsigned char *operations, size_t operation_size,
           unsigned char *payload, size_t payload_size)
{
 // Frame one direct command, transfer the complete reply, validate its ID/status/size, and copy globals; return 0 or -1.
 unsigned char packet[64], response[64];
 unsigned int message_id=(unsigned int)message_id_counter++ & 0xffff;
 size_t packet_size=7+operation_size;
 if (packet_size>sizeof(packet)||payload_size>sizeof(response)-3) return -1;
 // Header: little-endian length excluding itself, message ID, reply request, and global-memory allocation.
 packet[0]=(unsigned char)(packet_size-2);
 packet[1]=0;
 packet[2]=(unsigned char)message_id;
 packet[3]=(unsigned char)(message_id>>8);
 packet[4]=0;
 packet[5]=(unsigned char)payload_size;
 packet[6]=0;
 memcpy(packet+7,operations,operation_size);
 // Partial transfers are legal; retry interrupted calls without resending bytes already delivered.
 size_t transferred=0;
 while (transferred<packet_size)
 {
  ssize_t count=send(*socket_id,packet+transferred,packet_size-transferred,MSG_NOSIGNAL);
  if (count<0&&errno==EINTR) continue;
  if (count<=0) return -1;
  transferred+=(size_t)count;
 }
 // RFCOMM is a byte stream: read the length prefix before collecting the complete reply body.
 unsigned char length_bytes[2];
 transferred=0;
 while (transferred<2)
 {
  ssize_t count=recv(*socket_id,length_bytes+transferred,2-transferred,0);
  if (count<0&&errno==EINTR) continue;
  if (count<=0) return -1;
  transferred+=(size_t)count;
 }
 size_t response_size=length_bytes[0]+256u*length_bytes[1];
 // Three reply bytes hold the message ID and status; the rest must match the requested globals.
 if (response_size!=3+payload_size) return -1;
 transferred=0;
 while (transferred<response_size)
 {
  ssize_t count=recv(*socket_id,response+transferred,response_size-transferred,0);
  if (count<0&&errno==EINTR) continue;
  if (count<=0) return -1;
  transferred+=(size_t)count;
 }
 // Never interpret data from another request or a firmware error as sensor measurements.
 if (response[0]!=(unsigned char)message_id||response[1]!=(unsigned char)(message_id>>8)||response[2]!=2)
  return -1;
 memcpy(payload,response+3,payload_size);
 return 0;
}

static int wait_for_calibration_enter(void)
{
 // Consume a whole terminal line so typed text cannot leak into the next colour prompt; return 0 on Enter, -1 on EOF.
 int character;
 while ((character=getchar())!=EOF)
  if (character=='\n') return 0;
 return -1;
}

static int verify_nxt_colour_sensor(void)
{
 unsigned char identify[]={opINPUT_DEVICE,GET_TYPEMODE,0,COLOUR_SENSOR_PORT,0x60,0x61};
 unsigned char device[2];
 if (calibration_command(identify,sizeof(identify),device,2)!=0||device[0]!=4)
 {
  fprintf(stderr,"Calibration requires an NXT colour sensor on input %d.\n",COLOUR_SENSOR_PORT+1);
  return -1;
 }
 return 0;
}

#define CALIBRATION_SAMPLES_PER_COLOUR 30
static const char *const calibration_colours[]={"red","green","blue","yellow","black","white"};
static const int calibration_map_ids[]={5,3,2,4,1,6};
static int colour_calibration_samples[6][CALIBRATION_SAMPLES_PER_COLOUR][3];
static int colour_calibration_loaded=0;
static int confirmed_colour=0;               // Last colour confirmed by three consecutive readings.
static int candidate_colour=0;               // Colour currently accumulating a confirmation streak.
static int candidate_count=0;                // Consecutive matches, capped at three.

void reset_sensor_color_filter(void)
{
 // Discard all temporal history, such as when switching calibration or starting a new scan.
 confirmed_colour=0;
 candidate_colour=0;
 candidate_count=0;
}

int load_colour_calibration(const char *directory)
{
 // Load six complete NXT captures for nearest-sample classification.
 colour_calibration_loaded=0;
 reset_sensor_color_filter();
 for (int colour_index=0;colour_index<6;colour_index++)
 {
  char filename[1024],header[128];
  int length=snprintf(filename,sizeof(filename),"%s/%s.txt",directory,calibration_colours[colour_index]);
  if (length<0||(size_t)length>=sizeof(filename)) return -1;
  FILE *input=fopen(filename,"r");
  if (input==NULL) return -1;
  if (fgets(header,sizeof(header),input)==NULL||strcmp(header,"sensor NXT RGB\n")!=0||
      fgets(header,sizeof(header),input)==NULL||strcmp(header,"sample R G B ambient\n")!=0)
  {
   fclose(input);
   return -1;
  }
  // Reject EV3 files, incomplete captures, and impossible corrected NXT readings.
  int failed=0;
  for (int sample=1;sample<=CALIBRATION_SAMPLES_PER_COLOUR;sample++)
  {
   int index,red,green,blue,ambient;
   if (fscanf(input,"%d %d %d %d %d",&index,&red,&green,&blue,&ambient)!=5||index!=sample||
    red< -32767||red>32767||green< -32767||green>32767||blue< -32767||blue>32767||ambient<0||ambient>32767)
   {
    failed=1;
    break;
   }
   colour_calibration_samples[colour_index][sample-1][0]=red;
   colour_calibration_samples[colour_index][sample-1][1]=green;
   colour_calibration_samples[colour_index][sample-1][2]=blue;
  }
  // Allow trailing whitespace, but reject extra rows or text from an incompatible file format.
  int character;
  while (!failed&&(character=fgetc(input))!=EOF)
   if (!isspace((unsigned char)character)) failed=1;
  if (ferror(input)) failed=1;
  fclose(input);
  if (failed) return -1;
 }
 // Enable classification only after all six NXT calibration files validate.
 colour_calibration_loaded=1;
 return 0;
}

static int classify_sensor_rgb(const int channels[3])
{
 // Require a clear nearest-class margin; ambiguous RGB values stay unknown instead of crossing colour boundaries.
 if (!colour_calibration_loaded) return 0;
 double colour_distances[7];
 for (int colour=0;colour<7;colour++) colour_distances[colour]=HUGE_VAL;
 for (int colour_index=0;colour_index<6;colour_index++)
  for (int sample=0;sample<CALIBRATION_SAMPLES_PER_COLOUR;sample++)
  {
   double distance=0;
   for (int channel=0;channel<3;channel++)
   {
    double difference=(double)channels[channel]-colour_calibration_samples[colour_index][sample][channel];
    distance+=difference*difference;
   }
   int colour=calibration_map_ids[colour_index];
   if (distance<colour_distances[colour]) colour_distances[colour]=distance;
  }

 double nearest_distance=HUGE_VAL,second_distance=HUGE_VAL;
 int nearest_colour=0;
 for (int colour=1;colour<=6;colour++)
 {
  if (colour_distances[colour]<nearest_distance)
   {
   second_distance=nearest_distance;
   nearest_distance=colour_distances[colour];
   nearest_colour=colour;
   }
  else if (colour_distances[colour]<second_distance)
   {
   second_distance=colour_distances[colour];
   }
 }
 if (nearest_distance>=second_distance*COLOUR_CLASSIFICATION_MAX_DISTANCE_RATIO) return 0;
 return nearest_colour;
}

static int read_sensor_colour_unfiltered(void)
{
 if (!colour_calibration_loaded) return -1;
 struct timeval timeout={5,0};
 if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
   setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0) return -1;
 int rgb[3],ambient;
 if (BT_read_colour_RGBraw_NXT(COLOUR_SENSOR_PORT,&rgb[0],&rgb[1],&rgb[2],&ambient)!=1||
   rgb[0]< -32767||rgb[0]>32767||rgb[1]< -32767||rgb[1]>32767||rgb[2]< -32767||rgb[2]>32767||
   ambient<0||ambient>32767) return -1;
 return classify_sensor_rgb(rgb);
}

static int sample_colour_agreement(int required_votes)
{
 int counts[7]={0};
 for (int sample=0;sample<5;sample++)
 {
  int colour=read_sensor_colour_unfiltered();
  if (colour<0) return -1;
  if (colour>=1&&colour<=6) counts[colour]++;
  if (sample<4) usleep(20000);
 }
 int best_colour=0;
 for (int colour=1;colour<=6;colour++)
  if (counts[colour]>counts[best_colour]) best_colour=colour;
 printf("Alignment sample: %s (%d), %d/5\n",get_colour_name(best_colour),best_colour,counts[best_colour]);
 fflush(stdout);
 return counts[best_colour]>=required_votes ? best_colour : 0;
}

static int sample_stable_colour(void)
{
 return sample_colour_agreement(3);
}

static int sample_street_event_colour(void)
{
 int colour=sample_colour_agreement(4);
 if (colour<0) return -1;
 if (colour==5)
 {
  if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return -1;
  int confirmation=sample_colour_agreement(3);
  if (confirmation!=5)
  {
   printf("Stationary red-boundary confirmation failed; treating it as unknown.\n");
   colour=0;
  }
 }
 printf("Street sample: %s (%d)\n",get_colour_name(colour),colour);
 fflush(stdout);
 return colour;
}

static int sample_alignment_colour(void)
{
 return sample_colour_agreement(4);
}

static int sample_sweep_colour(void)
{
 int counts[7]={0};
 for (int sample=0;sample<5;sample++)
 {
  int colour=read_sensor_colour_unfiltered();
  if (colour<0) return -1;
  if (colour>=1&&colour<=6) counts[colour]++;
  if (sample<4) usleep(20000);
 }
 int best_colour=0,best_count=0,runner_up=0;
 for (int colour=1;colour<=6;colour++)
 {
  if (counts[colour]>best_count)
  {
   runner_up=best_count;
   best_colour=colour;
   best_count=counts[colour];
  }
  else if (counts[colour]>runner_up) runner_up=counts[colour];
 }
 int required_votes=(best_colour==1||best_colour==4) ? 2 : 4;
 printf("Sweep sample: %s (%d), %d/5\n",get_colour_name(best_colour),best_colour,best_count);
 fflush(stdout);
 return best_count>=required_votes&&best_count>runner_up ? best_colour : 0;
}

static int continuous_black_sweep(int *readings,int sample_count,int *final_angle)
{
 int angle,rate;
 if (BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1) return -1;
 int initial_colour=sample_sweep_colour();
 if (initial_colour<0) return -1;
 readings[0]=initial_colour;
 if (BT_turn(MOTOR_C,ALIGN_SWEEP_POWER,MOTOR_B,-ALIGN_SWEEP_POWER)!=0)
 {
  BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
  return -1;
 }

 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 struct timespec last_progress_time;
 clock_gettime(CLOCK_MONOTONIC,&last_progress_time);
 int last_progress_angle=0;
 int next_sample_angle=ALIGN_SWEEP_STEP_DEGREES;
 int read_error=0;
 while (next_sample_angle<ALIGN_SWEEP_TOTAL_DEGREES)
 {
  if (elapsed_ms(&start)>=ALIGN_SWEEP_TIMEOUT_MS)
  {
   fprintf(stderr,"Continuous alignment sweep timed out.\n");
   read_error=1;
   break;
  }
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1)
  {
   read_error=1;
   break;
  }
  if (angle!=last_progress_angle)
  {
   last_progress_angle=angle;
   clock_gettime(CLOCK_MONOTONIC,&last_progress_time);
  }
  else if (elapsed_ms(&last_progress_time)>=5000)
  {
   fprintf(stderr,"Alignment sweep stalled at gyro %d degrees before target %d; stopping motors.\n",
     angle,next_sample_angle);
   read_error=1;
   break;
  }
  if (angle<next_sample_angle)
  {
   usleep(10000);
   continue;
  }

  int colour=sample_sweep_colour();
  if (colour<0)
  {
   read_error=1;
   break;
  }
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1)
  {
   read_error=1;
   break;
  }
  int sample_index=(int)lround((double)angle/ALIGN_SWEEP_STEP_DEGREES)%sample_count;
  readings[sample_index]=colour;
  printf("Sweep: gyro %d degrees, %s (%d)\n",angle,get_colour_name(colour),colour);
  fflush(stdout);
  next_sample_angle=(angle/ALIGN_SWEEP_STEP_DEGREES+1)*ALIGN_SWEEP_STEP_DEGREES;
 }

 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return -1;
 if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return -1;
 if (read_error||angle<ALIGN_SWEEP_TOTAL_DEGREES-ALIGN_SWEEP_STEP_DEGREES)
 {
  fprintf(stderr,"Alignment sweep ended at %d degrees, before a complete turn.\n",angle);
  return -1;
 }
 int overshoot=angle-ALIGN_SWEEP_TOTAL_DEGREES;
 if (overshoot>3)
 {
  if (overshoot>45)
  {
   fprintf(stderr,"Alignment sweep overshot by %d degrees; refusing a large correction.\n",overshoot);
   return -1;
  }
  printf("Sweep overshot by %d degrees; correcting back to 360.\n",overshoot);
  MoveResult correction=turn_in_place(overshoot,1,10000);
  if (correction.reason!=STOP_ANGLE)
  {
   report_move_result("alignment sweep overshoot correction",correction);
   return -1;
  }
  *final_angle=ALIGN_SWEEP_TOTAL_DEGREES;
 }
 else
  *final_angle=angle;
 return 0;
}

static int update_sensor_color_filter(int colour)
{
 // Debounce colour changes across calls: retain the confirmed ID until a new candidate appears three times in a row.
 if (colour<=0)
 {
  // Unknown breaks the streak but holds the confirmed colour; errors also break it and remain visible as -1.
  candidate_colour=0;
  candidate_count=0;
  return colour<0 ? -1 : confirmed_colour;
 }
 if (colour!=candidate_colour)
 {
  // A different candidate must start its own streak, not inherit matches from the previous colour.
  candidate_colour=colour;
  candidate_count=0;
 }
 if (candidate_count<3) candidate_count++;
 if (candidate_count==3) confirmed_colour=colour;
 return confirmed_colour;
}

int get_sensor_color_robust(void)
{
 // Read NXT corrected RGB, classify against saved samples, confirm three times, and print every request.
 int colour;
 if (!colour_calibration_loaded) colour=-1;
 else
 {
  struct timeval timeout={5,0};
  if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
      setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0)
   colour=update_sensor_color_filter(-1);
  else
  {
    int rgb[3],ambient;
    if (BT_read_colour_RGBraw_NXT(COLOUR_SENSOR_PORT,&rgb[0],&rgb[1],&rgb[2],&ambient)!=1||
       rgb[0]< -32767||rgb[0]>32767||rgb[1]< -32767||rgb[1]>32767||rgb[2]< -32767||rgb[2]>32767||ambient<0||ambient>32767)
     colour=update_sensor_color_filter(-1);
    else
     colour=update_sensor_color_filter(classify_sensor_rgb(rgb));
  }
 }
 printf("Seeing colour: %s (%d)\n",get_colour_name(colour),colour);
 fflush(stdout);
 return colour;
}

int calibrate_sensor(void)
{
 // On an open EV3 connection, save 30 NXT RGB+A readings per colour across three spots.
 struct timeval timeout={5,0};
 if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
     setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0)
 {
  perror("Unable to set calibration communication timeout");
  return -1;
 }
 if (verify_nxt_colour_sensor()!=0) return -1;
 if (mkdir("calibration",0755)!=0&&errno!=EEXIST)
 {
  perror("Unable to create calibration directory");
  return -1;
 }
 // Use the first unused numeric folder and preserve all earlier calibration runs.
 char directory[64];
 int run_number;
 for (run_number=1;run_number<1000000;run_number++)
 {
  snprintf(directory,sizeof(directory),"calibration/run-%d",run_number);
  if (mkdir(directory,0755)==0) break;
  if (errno!=EEXIST) {perror("Unable to create calibration session");return -1;}
 }
 if (run_number==1000000)
 {
  fprintf(stderr,"No unused calibration run number available.\n");
  return -1;
 }
 printf("Saving raw calibration readings in %s\n",directory);
 for (size_t colour_index=0;colour_index<6;colour_index++)
 {
  char filename[256];
   snprintf(filename,sizeof(filename),"%s/%s.txt",directory,calibration_colours[colour_index]);
  FILE *output=fopen(filename,"wx");
  if (output==NULL) {perror("Unable to create colour file");return -1;}
    int failed=fprintf(output,"sensor NXT RGB\nsample R G B ambient\n")<0;
    for (int sample=1;sample<=CALIBRATION_SAMPLES_PER_COLOUR&&!failed;sample++)
  {
   if ((sample-1)%10==0)
   {
        printf("Now reading %s: place sensor at distinct solid-colour spot %d/3 and press Enter.\n",
          calibration_colours[colour_index],(sample-1)/10+1);
    fflush(stdout);
    if (wait_for_calibration_enter()!=0) {failed=1;break;}
   }
  int channels[3],ambient;
  if (BT_read_colour_RGBraw_NXT(COLOUR_SENSOR_PORT,&channels[0],&channels[1],&channels[2],&ambient)!=1)
   {
    fprintf(stderr,"Sensor read failed; %s is incomplete.\n",filename);
    failed=1;
    break;
   }
  for (int channel=0;channel<3;channel++)
   if (channels[channel]< -32767||channels[channel]>32767) failed=1;
  if (ambient<0||ambient>32767) failed=1;
  failed=fprintf(output,"%d %d %d %d %d\n",sample,channels[0],channels[1],channels[2],ambient)<0||failed;
   if (fflush(output)!=0) failed=1;
  printf("%s %d/30: R=%d G=%d B=%d A=%d\n",calibration_colours[colour_index],sample,channels[0],channels[1],channels[2],ambient);
   if (sample%10!=0) usleep(100000);
   else if (!failed&&beep()!=0) failed=1;
  }
  // Signal completion only after all rows have been written and the file has closed successfully.
  if (fclose(output)!=0) failed=1;
  if (failed) {fprintf(stderr,"Calibration stopped; check %s.\n",filename);return -1;}
  if (beep()!=0) {fprintf(stderr,"Completion beep failed.\n");return -1;}
  printf("Saved 30 NXT RGB+A readings to %s\n",filename);
 }
 printf("NXT calibration capture complete.\n");
 return 0;
}

// ---- Smooth movement: gyro-held straight drives and gyro-controlled turns ----
#define MOVE_CRUISE_POWER 8
#define MOVE_MIN_POWER 12
#define MOVE_RAMP_UP_MS 500
#define MOVE_RAMP_DOWN_DEGREES 90.0
#define MOVE_HEADING_GAIN 1.5
#define MOVE_HEADING_LIMIT 8
#define TURN_MIN_POWER 10
#define TURN_MAX_POWER 20
#define TURN_GAIN 0.6
#define TURN_TOLERANCE_DEGREES 3
#define TURN_SETTLE_US 150000
#define TURN_DEFAULT_TIMEOUT_MS 10000

enum {ROTATE_BOTH=0, ROTATE_LEFT_WHEEL=1, ROTATE_RIGHT_WHEEL=2};

static long elapsed_ms(const struct timespec *start)
{
 struct timespec now;
 clock_gettime(CLOCK_MONOTONIC,&now);
 return (now.tv_sec-start->tv_sec)*1000L+(now.tv_nsec-start->tv_nsec)/1000000L;
}

static MoveResult stop_with(StopReason reason,int colour)
{
 MoveResult result={reason,colour};
 // Always brake, including on errors, so a failed read can never leave the wheels running.
 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) result.reason=STOP_ERROR;
 return result;
}

static int movement_setup(const int *colours,int count)
{
 if (count<0||(count>0&&colours==NULL)) return -1;
 for (int i=0;i<count;i++)
  if (colours[i]<1||colours[i]>6) return -1;
 struct timeval timeout={5,0};
 if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
     setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0) return -1;
 // Require fresh confirmations so a colour seen before the move cannot trigger an immediate stop.
 reset_sensor_color_filter();
 return 0;
}

static int read_wheel_degrees(int *right,int *left)
{
 // Output indexes: A=0, B=1 (right wheel), C=2 (left wheel); counts are 32-bit little endian.
 const unsigned char operations[]={opOUTPUT_GET_COUNT,LC0(0),LC0(1),GV0(0),opOUTPUT_GET_COUNT,LC0(0),LC0(2),GV0(4)};
 unsigned char payload[8];
 if (calibration_command(operations,sizeof(operations),payload,sizeof(payload))!=0) return -1;
 *right=(int32_t)((uint32_t)payload[0]|(uint32_t)payload[1]<<8|(uint32_t)payload[2]<<16|(uint32_t)payload[3]<<24);
 *left=(int32_t)((uint32_t)payload[4]|(uint32_t)payload[5]<<8|(uint32_t)payload[6]<<16|(uint32_t)payload[7]<<24);
 return 0;
}

static int hit_stop_colour(const int *colours,int count)
{
 // Returns the matching colour ID, 0 for no match, -1 on a read error.
 int colour=get_sensor_color_robust();
 if (colour<0) return -1;
 for (int i=0;i<count;i++)
  if (colours[i]==colour) return colour;
 return 0;
}

static int ramp_power(int target_power,long elapsed)
{
 // Linear rise from the minimum power so the wheels never jerk up to speed.
 if (elapsed>=MOVE_RAMP_UP_MS) return target_power;
 return MOVE_MIN_POWER+(int)((target_power-MOVE_MIN_POWER)*elapsed/MOVE_RAMP_UP_MS);
}

static int clamp_power(int power)
{
 return power>100 ? 100 : (power<-100 ? -100 : power);
}

static MoveResult drive_straight(int direction,const int *stop_colours,int colour_count,double max_revolutions,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (movement_setup(stop_colours,colour_count)!=0||(max_revolutions<=0&&timeout_ms<=0)) return failed;
 int angle,rate,right0,left0;
 if (BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1||read_wheel_degrees(&right0,&left0)!=0) return failed;
 double limit=max_revolutions>0 ? max_revolutions*360.0 : 0;
 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 for (;;)
 {
  long elapsed=elapsed_ms(&start);
  if (timeout_ms>0&&elapsed>=timeout_ms) return stop_with(STOP_TIMEOUT,0);
  int right,left;
  if (read_wheel_degrees(&right,&left)!=0) return stop_with(STOP_ERROR,0);
  double travelled=(abs(right-right0)+abs(left-left0))/2.0;
  if (limit>0&&travelled>=limit) return stop_with(STOP_DISTANCE,0);
  if (colour_count>0)
  {
   int colour=get_sensor_color_robust();
   if (colour<0) return stop_with(STOP_ERROR,0);
   for (int stop_index=0;stop_index<colour_count;stop_index++)
    if (colour==stop_colours[stop_index]) return stop_with(STOP_COLOUR,colour);
  }
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return stop_with(STOP_ERROR,0);
  int magnitude=ramp_power(MOVE_CRUISE_POWER,elapsed);
  if (limit>0&&limit-travelled<MOVE_RAMP_DOWN_DEGREES)
  {
   // Slow smoothly over the final stretch of a distance-limited move.
   int tapered=MOVE_MIN_POWER+(int)((MOVE_CRUISE_POWER-MOVE_MIN_POWER)*(limit-travelled)/MOVE_RAMP_DOWN_DEGREES);
   if (tapered<magnitude) magnitude=tapered;
  }
  // Gyro angle grows clockwise; a positive angle is drift to the right, so speed up the right wheel.
  int correction=(int)(MOVE_HEADING_GAIN*angle);
  if (correction>MOVE_HEADING_LIMIT) correction=MOVE_HEADING_LIMIT;
  if (correction<-MOVE_HEADING_LIMIT) correction=-MOVE_HEADING_LIMIT;
  int base=direction*magnitude;
  if (BT_turn(MOTOR_C,clamp_power(base-correction),MOTOR_B,clamp_power(base+correction))!=0)
   return stop_with(STOP_ERROR,0);
 }
}

static MoveResult rotate_gyro_with_power_limits(int wheels,int turn_right,int angle_degrees,
                                                const int *stop_colours,int colour_count,
                                                int timeout_ms,int min_power,int max_power)
{
 MoveResult failed={STOP_ERROR,0};
 if (angle_degrees<1||angle_degrees>360||movement_setup(stop_colours,colour_count)!=0) return failed;
 // A turn always has a bound, so a dead gyro cannot spin the robot forever.
 if (timeout_ms<=0) timeout_ms=TURN_DEFAULT_TIMEOUT_MS;
 int angle,rate;
 if (BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1) return failed;
 int target=turn_right ? angle_degrees : -angle_degrees;
 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 for (;;)
 {
  long elapsed=elapsed_ms(&start);
  if (elapsed>=timeout_ms) return stop_with(STOP_TIMEOUT,0);
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return stop_with(STOP_ERROR,0);
  if (colour_count>0)
  {
   int colour=hit_stop_colour(stop_colours,colour_count);
   if (colour<0) return stop_with(STOP_ERROR,0);
   if (colour>0) return stop_with(STOP_COLOUR,colour);
  }
  int error=target-angle;
  if (abs(error)<=TURN_TOLERANCE_DEGREES)
  {
   // Brake, let momentum settle, and accept only if the heading is still on target.
   MoveResult braked=stop_with(STOP_ANGLE,0);
   if (braked.reason!=STOP_ANGLE) return braked;
   usleep(TURN_SETTLE_US);
   if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return failed;
   if (abs(target-angle)<=TURN_TOLERANCE_DEGREES) return braked;
   continue;
  }
  // Power tapers with the remaining error; a negative error (overshoot) reverses the wheels to correct.
  int magnitude=(int)(TURN_GAIN*abs(error));
  int ceiling=elapsed>=MOVE_RAMP_UP_MS ? max_power :
    min_power+(max_power-min_power)*elapsed/MOVE_RAMP_UP_MS;
  if (magnitude<min_power) magnitude=min_power;
  if (magnitude>ceiling) magnitude=ceiling;
  int direction=error>0 ? 1 : -1;
  // Clockwise = left wheel forward and/or right wheel backward.
  int left=wheels!=ROTATE_RIGHT_WHEEL ? direction*magnitude : 0;
  int right=wheels!=ROTATE_LEFT_WHEEL ? -direction*magnitude : 0;
  if (BT_turn(MOTOR_C,left,MOTOR_B,right)!=0) return stop_with(STOP_ERROR,0);
 }
}

static MoveResult rotate_gyro(int wheels,int turn_right,int angle_degrees,const int *stop_colours,
                              int colour_count,int timeout_ms)
{
 return rotate_gyro_with_power_limits(wheels,turn_right,angle_degrees,stop_colours,colour_count,
                                      timeout_ms,TURN_MIN_POWER,TURN_MAX_POWER);
}

MoveResult move_forward_until(const int *stop_colours,int colour_count,double max_revolutions,int timeout_ms)
{
 return drive_straight(1,stop_colours,colour_count,max_revolutions,timeout_ms);
}

MoveResult move_backward_until(const int *stop_colours,int colour_count,double max_revolutions,int timeout_ms)
{
 return drive_straight(-1,stop_colours,colour_count,max_revolutions,timeout_ms);
}

MoveResult pivot_single_wheel(int angle_degrees,int turn_direction,int reverse,const int *stop_colours,int colour_count,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (turn_direction!=0&&turn_direction!=1) return failed;
 int turn_right=turn_direction==0;
 // Left wheel drives a right-forward or left-reverse pivot; the right wheel drives the other two.
 int wheels=turn_right!=(reverse!=0) ? ROTATE_LEFT_WHEEL : ROTATE_RIGHT_WHEEL;
 return rotate_gyro(wheels,turn_right,angle_degrees,stop_colours,colour_count,timeout_ms);
}

MoveResult turn_in_place(int angle_degrees,int turn_direction,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (turn_direction!=0&&turn_direction!=1) return failed;
 return rotate_gyro(ROTATE_BOTH,turn_direction==0,angle_degrees,NULL,0,timeout_ms);
}

static MoveResult turn_in_place_alignment(int angle_degrees,int turn_direction,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (turn_direction!=0&&turn_direction!=1) return failed;
 return rotate_gyro_with_power_limits(ROTATE_BOTH,turn_direction==0,angle_degrees,NULL,0,
                                      timeout_ms,ALIGN_TURN_MIN_POWER,ALIGN_TURN_MAX_POWER);
}

MoveResult turn_in_place_until(int angle_degrees,int turn_direction,const int *stop_colours,int colour_count,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (turn_direction!=0&&turn_direction!=1) return failed;
 return rotate_gyro(ROTATE_BOTH,turn_direction==0,angle_degrees,stop_colours,colour_count,timeout_ms);
}

int report_move_result(const char *label,MoveResult result)
{
 // Prints why a move ended; returns 1 for an error so callers can abort a sequence.
 switch (result.reason)
 {
  case STOP_TIMEOUT: printf("%s: stopped by timeout\n",label); break;
  case STOP_DISTANCE: printf("%s: stopped by distance limit\n",label); break;
  case STOP_COLOUR: printf("%s: stopped by colour %s (%d)\n",label,get_colour_name(result.colour),result.colour); break;
  case STOP_ANGLE: printf("%s: angle reached\n",label); break;
  default: fprintf(stderr,"%s: error, motors braked\n",label); return 1;
 }
 return 0;
}

static int continuous_recovery_sweep(void)
{
 return window_wipe_for_colour(0);
}

static int recover_black_heading(void)
{
 printf("Line lost; stopping, sweeping left 30 degrees, then right through to +30 degrees.\n");
 int result=continuous_recovery_sweep();
 if (result==4)
 {
  printf("Yellow intersection found during recovery wiggle; stopping there.\n");
  return 4;
 }
 if (result==5)
 {
  printf("Red boundary found during recovery sweep.\n");
  return 5;
 }
 if (result!=1)
 {
  fprintf(stderr,"Black line not found in the bounded recovery sweep.\n");
  return 0;
 }
 if (!center_on_alignment_marker(1)) return 0;
 int angle,rate;
 if (BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1) return 0;
 printf("Black reacquired; heading reset. Resuming forward.\n");
 return 1;
}

static int measure_marker_edge(int turn_direction,int marker_colour,int *edge_degrees,int *swept_degrees)
{
 int turned=0;
 int first_nonblack=0,nonblack_count=0;
 while (turned<ROAD_CENTER_MAX_SWEEP_DEGREES)
 {
  MoveResult turn=turn_in_place(ROAD_CENTER_STEP_DEGREES,turn_direction,5000);
  if (turn.reason!=STOP_ANGLE) return -1;
  turned+=ROAD_CENTER_STEP_DEGREES;
  *swept_degrees=turned;
  int colour=sample_alignment_colour();
  if (colour<0) return -1;
  if (colour==marker_colour)
  {
   nonblack_count=0;
   continue;
  }
  if (nonblack_count==0) first_nonblack=turned;
  nonblack_count++;
  if (nonblack_count==2)
  {
   *edge_degrees=first_nonblack-ROAD_CENTER_STEP_DEGREES;
   *swept_degrees=turned;
   return 1;
  }
 }
 *swept_degrees=turned;
 return 0;
}

static int center_on_alignment_marker(int marker_colour)
{
 int colour=sample_alignment_colour();
 if (colour<0) return 0;
 if (colour!=marker_colour)
 {
  printf("Searching gently for %s from %s before measuring its edges.\n",
    get_colour_name(marker_colour),get_colour_name(colour));
  int acquired=0;
  for (int direction=1;direction>=0&&!acquired;direction--)
  {
   int turned=0;
   while (turned<ROAD_CENTER_MAX_SWEEP_DEGREES)
   {
    MoveResult search_turn=turn_in_place(ROAD_CENTER_STEP_DEGREES,direction,10000);
    if (search_turn.reason!=STOP_ANGLE) return 0;
    turned+=ROAD_CENTER_STEP_DEGREES;
    colour=sample_alignment_colour();
    if (colour<0) return 0;
    if (colour==marker_colour)
    {
     acquired=1;
     printf("Found %s after %s %d degrees.\n",get_colour_name(marker_colour),
       direction==1 ? "left" : "right",turned);
     break;
    }
   }
   if (!acquired&&turned>0)
   {
    MoveResult restore=turn_in_place(turned,direction==1 ? 0 : 1,10000);
    if (restore.reason!=STOP_ANGLE) return 0;
   }
  }
  if (!acquired)
  {
   fprintf(stderr,"Could not find %s within the gentle search range.\n",
     get_colour_name(marker_colour));
   return 0;
  }
 }

 int left_edge=0,left_swept=0;
 int left_status=measure_marker_edge(1,marker_colour,&left_edge,&left_swept);
 if (left_swept>0)
 {
  MoveResult restore=turn_in_place(left_swept,0,10000);
  if (restore.reason!=STOP_ANGLE) return 0;
 }
 if (left_status<0) return 0;
 if (left_status==0)
 {
  printf("Could not find the left edge of %s within %d degrees; keeping current heading.\n",
    get_colour_name(marker_colour),ROAD_CENTER_MAX_SWEEP_DEGREES);
  return 1;
 }

 int right_edge=0,right_swept=0;
 int right_status=measure_marker_edge(0,marker_colour,&right_edge,&right_swept);
 if (right_status!=1)
 {
  if (right_swept>0)
  {
   MoveResult restore=turn_in_place(right_swept,1,20000);
   if (restore.reason!=STOP_ANGLE) return 0;
  }
  if (right_status<0) return 0;
  printf("Could not find the right edge of %s within %d degrees; keeping current heading.\n",
    get_colour_name(marker_colour),ROAD_CENTER_MAX_SWEEP_DEGREES);
  return 1;
 }

 int center_offset=(right_edge-left_edge)/2;
 int correction=center_offset-right_swept;
 printf("%s bounds: left -%d, right +%d degrees; centering by %d degrees.\n",
   get_colour_name(marker_colour),left_edge,right_edge,abs(correction));
 if (correction!=0)
 {
  MoveResult center=turn_in_place(abs(correction),correction>0 ? 0 : 1,20000);
  if (center.reason!=STOP_ANGLE) return 0;
 }
 if (sample_alignment_colour()!=marker_colour)
 {
  fprintf(stderr,"Marker midpoint did not verify %s; using target-color wipe.\n",
    get_colour_name(marker_colour));
  if (window_wipe_for_colour(marker_colour)!=marker_colour) return 0;
 }
 else
  printf("Centered on %s marker.\n",get_colour_name(marker_colour));
 return 1;
}

static int follow_black_street(void)
{
 const double max_forward_revolutions=2;
 const int timeout_ms=60000;
 const int follow_power=10;
 const int steering_correction_limit=4;
 int angle,rate,right_previous,left_previous;
 int alignment_colour=next_follow_alignment_colour;
 next_follow_alignment_colour=1;
 if (!center_on_alignment_marker(alignment_colour)) return 0;
 if (movement_setup(NULL,0)!=0||BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1||
   read_wheel_degrees(&right_previous,&left_previous)!=0) return 0;
 double forward_revolutions=0;
 int last_healthy_colour=1;
 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 printf("Following black street until yellow or red.\n");

 for (;;)
 {
  if (elapsed_ms(&start)>=timeout_ms)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   fprintf(stderr,"Street following timed out before a marker.\n");
   return 0;
  }
  int right,left;
  if (read_wheel_degrees(&right,&left)!=0||BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   return 0;
  }
  forward_revolutions+=(abs(right-right_previous)+abs(left-left_previous))/720.0;
  right_previous=right;
  left_previous=left;
  if (forward_revolutions>=max_forward_revolutions)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   fprintf(stderr,"No intersection or boundary within one cell.\n");
   return 0;
  }

  int colour=sample_street_event_colour();
  if (colour==0)
  {
   colour=last_healthy_colour;
   printf("Unknown street reading; retaining last healthy colour %s.\n",
     get_colour_name(last_healthy_colour));
  }
  if (colour==4)
  {
   stop_with(STOP_COLOUR,colour);
    printf("Intersection reached; stopped on yellow.\n");
    return 1;
  }
  if (colour==5)
  {
   stop_with(STOP_COLOUR,colour);
   printf("Red boundary reached.\n");
   return 2;
  }
  if (colour==1)
  {
    last_healthy_colour=colour;
    // Stay on the gyro-held heading while the sensor confirms the road.
  }
  else
  {
   if (colour<0)
   {
    fprintf(stderr,"Colour sensor read failed while following street.\n");
    BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
    return 0;
   }
    if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return 0;
    printf("Black lost (%s); starting bounded left/right wiggle.\n",get_colour_name(colour));
    int recovery= recover_black_heading();
    if (recovery==4) return 1;
    if (recovery==5) return 2;
    if (recovery!=1) return 0;
    if (read_wheel_degrees(&right_previous,&left_previous)!=0)
    {
     BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
     return 0;
    }
    continue;
  }

  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   return 0;
  }
    int correction=(int)(MOVE_HEADING_GAIN*angle);
    if (correction>steering_correction_limit) correction=steering_correction_limit;
    if (correction< -steering_correction_limit) correction=-steering_correction_limit;
    if (BT_turn(MOTOR_C,follow_power-correction,
      MOTOR_B,follow_power+correction)!=0)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   return 0;
  }
 }
}

int parse_map(unsigned char *map_img, int rx, int ry)
{
 /*
   This function takes an input image map array, and two integers that specify the image size.
   It attempts to parse this image into a representation of the map in the image. The size
   and resolution of the map image should not affect the parsing (i.e. you can make your own
   maps without worrying about the exact position of intersections, roads, buildings, etc.).

   However, this function requires:
   
   * White background for the image  [255 255 255]
   * Red borders around the map  [255 0 0]
   * Black roads  [0 0 0]
   * Yellow intersections  [255 255 0]
   * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
   (any other colour values are ignored - so you can add markings if you like, those 
    will not affect parsing)

   The image must be a properly formated .ppm image, see readPPMimage below for details of
   the format. The GIMP image editor saves properly formatted .ppm images, as does the
   imagemagick image processing suite.
   
   The map representation is read into the map array, with each row in the array corrsponding
   to one intersection, in raster order, that is, for a map with k intersections along its width:
   
    (row index for the intersection)
    
    0     1     2    3 ......   k-1
    
    k    k+1   k+2  ........    
    
    Each row will then contain the colour values for buildings around the intersection 
    clockwise from top-left, that is
    
    
    top-left               top-right
            
            intersection
    
    bottom-left           bottom-right
    
    So, for the first intersection (at row 0 in the map array)
    map[0][0] <---- colour for the top-left building
    map[0][1] <---- colour for the top-right building
    map[0][2] <---- colour for the bottom-right building
    map[0][3] <---- colour for the bottom-left building
    
    Color values for map locations are defined as follows (this agrees with what the
    EV3 sensor returns in indexed-colour-reading mode):
    
    1 -  Black
    2 -  Blue
    3 -  Green
    4 -  Yellow
    5 -  Red
    6 -  White
    
    If you find a 0, that means you're trying to access an intersection that is not on the
    map! Also note that in practice, because of how the map is defined, you should find
    only Green, Blue, or White around a given intersection.
    
    The map size (the number of intersections along the horizontal and vertical directions) is
    updated and left in the global variables sx and sy.

    Feel free to create your own maps for testing (you'll have to print them to a reasonable
    size to use with your bot).
    
 */    
 
 int last3[3];
 int x,y;
 unsigned char R,G,B;
 int ix,iy;
 int bx,by,dx,dy,wx,wy;         // Intersection geometry parameters
 int tgl;
 int idx;
 
 ix=iy=0;       // Index to identify the current intersection
 
 // Determine the spacing and size of intersections in the map
 tgl=0;
 for (int i=0; i<rx; i++)
 {
  for (int j=0; j<ry; j++)
  {
   R=*(map_img+((i+(j*rx))*3));
   G=*(map_img+((i+(j*rx))*3)+1);
   B=*(map_img+((i+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0)
   {
    // First intersection, top-left pixel. Scan right to find width and spacing
    bx=i;           // Anchor for intersection locations
    by=j;
    for (int k=i; k<rx; k++)        // Find width and horizontal distance to next intersection
    {
     R=*(map_img+((k+(by*rx))*3));
     struct timeval timeout={0,500000};
     B=*(map_img+((k+(by*rx))*3)+2);
     if (tgl==0&&(R!=255||G!=255||B!=0))
     {
      tgl=1;
      wx=k-i;
     }
     if (tgl==1&&R==255&&G==255&&B==0)
     {
      tgl=2;
      dx=k-i;
     }
    }
    for (int k=j; k<ry; k++)        // Find height and vertical distance to next intersection
    {
     R=*(map_img+((bx+(k*rx))*3));
     G=*(map_img+((bx+(k*rx))*3)+1);
     B=*(map_img+((bx+(k*rx))*3)+2);
     if (tgl==2&&(R!=255||G!=255||B!=0))
     {
      tgl=3;
      wy=k-j;
     }
     if (tgl==3&&R==255&&G==255&&B==0)
     {
      tgl=4;
      dy=k-j;
     }
    }
    
    if (tgl!=4)
    {
     fprintf(stderr,"Unable to determine intersection geometry!\n");
     return(0);
    }
    else break;
   }
  }
  if (tgl==4) break;
 }
  fprintf(stderr,"Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n",bx,by,wx,wy,dx,dy);

  sx=0;
  for (int i=bx+(wx/2);i<rx;i+=dx)
  {
   R=*(map_img+((i+(by*rx))*3));
   G=*(map_img+((i+(by*rx))*3)+1);
   B=*(map_img+((i+(by*rx))*3)+2);
   if (R==255&&G==255&&B==0) sx++;
  }

  sy=0;
  for (int j=by+(wy/2);j<ry;j+=dy)
  {
   R=*(map_img+((bx+(j*rx))*3));
   G=*(map_img+((bx+(j*rx))*3)+1);
   B=*(map_img+((bx+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0) sy++;
  }
  
  fprintf(stderr,"Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n",sx,sy);

  // Scan for building colours around each intersection
  idx=0;
  for (int j=0; j<sy; j++)
   for (int i=0; i<sx; i++)
   {
    x=bx+(i*dx)+(wx/2);
    y=by+(j*dy)+(wy/2);
    
    fprintf(stderr,"Intersection location: %d, %d\n",x,y);
    // Top-left
    x-=wx;
    y-=wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][0]=3;
    else if (R==0&&G==0&&B==255) map[idx][0]=2;
    else if (R==255&&G==255&&B==255) map[idx][0]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n",i,j,R,G,B);

    // Top-right
    x+=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][1]=3;
    else if (R==0&&G==0&&B==255) map[idx][1]=2;
    else if (R==255&&G==255&&B==255) map[idx][1]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n",i,j,R,G,B);

    // Bottom-right
    y+=2*wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][2]=3;
    else if (R==0&&G==0&&B==255) map[idx][2]=2;
    else if (R==255&&G==255&&B==255) map[idx][2]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n",i,j,R,G,B);
    
    // Bottom-left
    x-=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][3]=3;
    else if (R==0&&G==0&&B==255) map[idx][3]=2;
    else if (R==255&&G==255&&B==255) map[idx][3]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n",i,j,R,G,B);
    
    fprintf(stderr,"Colours for this intersection: %d, %d, %d, %d\n",map[idx][0],map[idx][1],map[idx][2],map[idx][3]);
    
    idx++;
   }

 return(1);  
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # One or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //

 FILE *f;
 unsigned char *im;
 char line[1024];
 int i;
 unsigned char *tmp;
 double *fRGB;

 im=NULL;
 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }
 fprintf(stderr,"%s\n",line);
 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fprintf(stderr,"%s",line);
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",rx,ry);                  // Read image size
 fprintf(stderr,"nx=%d, ny=%d\n\n",*rx,*ry);

 fgets(&line[0],9,f);  	                // Read the remaining header line
 fprintf(stderr,"%s\n",line);
 im=(unsigned char *)calloc((*rx)*(*ry)*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }
 fread(im,(*rx)*(*ry)*3*sizeof(unsigned char),1,f);
 fclose(f);

 return(im);    
}