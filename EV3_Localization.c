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
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---
 
  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather informatin about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours. 
 
   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.
   
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

#include "EV3_Localization.h"

extern int *socket_id;

int map[400][4];            // This holds the representation of the map, up to 20x20
                            // intersections, raster ordered, 4 building colours per
                            // intersection.
int sx, sy;                 // Size of the map (number of intersections along x and y)
double beliefs[400][4];     // Beliefs for each location and motion direction

int main(int argc, char *argv[])
{
 char mapname[1024];
 int dest_x, dest_y, rx, ry;
 unsigned char *map_image;
 
 memset(&map[0][0],0,400*4*sizeof(int));
 sx=0;
 sy=0;
 
 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y [calibration_directory]\n");
  fprintf(stderr,"    map_name - should correspond to a properly formatted .ppm map image\n");
  fprintf(stderr,"    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
  fprintf(stderr,"    calibration_directory - saved colour files to load once for get_sensor_color_robust()\n");
  exit(1);
 }
 strcpy(&mapname[0],argv[1]);
 dest_x=atoi(argv[2]);
 dest_y=atoi(argv[3]);

 if (dest_x==-1&&dest_y==-1)
 {
  if (BT_open(HEXKEY)!=0) return 1;
  int calibration_status=calibrate_sensor();
  BT_close();
  return calibration_status==0 ? 0 : 1;
 }

 if (argc>4&&load_colour_calibration(argv[4])!=0)
 {
  fprintf(stderr,"Unable to load colour calibration from %s.\n",argv[4]);
  return 1;
 }

 /******************************************************************************************************************
  * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
  *   read your calibration data for use in your localization code. Skip this if you are not using calibration
  * ****************************************************************************************************************/
 
 
 // Your code for reading any calibration information should not go below this line //
 
 map_image=readPPMimage(&mapname[0],&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to open specified map image\n");
  exit(1);
 }
 
 if (parse_map(map_image, rx, ry)==0)
 { 
  fprintf(stderr,"Unable to parse input image map. Make sure the image is properly formatted\n");
  free(map_image);
  exit(1);
 }

 if (dest_x<0||dest_x>=sx||dest_y<0||dest_y>=sy)
 {
  fprintf(stderr,"Destination location is outside of the map\n");
  free(map_image);
  exit(1);
 }

 // Initialize beliefs - uniform probability for each location and direction
 for (int j=0; j<sy; j++)
  for (int i=0; i<sx; i++)
  {
   beliefs[i+(j*sx)][0]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][1]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][2]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][3]=1.0/(double)(sx*sy*4);
  }

 // Open a socket to the EV3 for remote controlling the bot.
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
  fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
  free(map_image);
  exit(1);
 }

 fprintf(stderr,"All set, ready to go!\n");
 
/*******************************************************************************************************************************
 *
 *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
 *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
 * 
 *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
 *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
 *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
 *          and 0<=j<=sy-1), index=i+(j*sx)
 *  
 *          In the beliefs[][] array, you need to keep track of 4 values per intersection, these correspond to the belief the
 *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
 * 
 *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
 *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
 *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
 *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
 * 
 *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
 *          belief values based on agreement between what the robot sensed, and the colours in the map. 
 * 
 *          You have two main tasks these are organized into two major functions:
 * 
 *          robot_localization()    <---- Runs the localization loop until the robot's location is found
 *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
 * 
 *          The target location, read from the command line, is left in dest_x, dest_y
 * 
 *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
 *          that even if your bot managed to find its location, it can become lost again while driving to the target
 *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place - 
 *          a very solid implementation should give your robot the ability to determine it's lost and needs to 
 *          run localization again.
 *
 *******************************************************************************************************************************/  

 // HERE - write code to call robot_localization() and go_to_target() as needed, any additional logic required to get the
 //        robot to complete its task should be here.


 int sensor_status=0;
 if (argc<5)
 {
  fprintf(stderr,"Pass a calibration directory to run the colour sensor loop.\n");
  sensor_status=1;
 }
 else
 {
  printf("Reading colours; press Ctrl+C to stop.\n");
  for (;;)
  {
   int colour=get_sensor_color_robust();
   if (colour<0)
   {
    fprintf(stderr,"Colour sensor read failed; stopping.\n");
    sensor_status=1;
    break;
   }
  printf("Colour: %s (%d)\n",get_colour_name(colour),colour);
   fflush(stdout);
   usleep(100000);
  }
 }
 // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
 BT_close();
 free(map_image);
 exit(sensor_status);
}

const char *get_colour_name(int colour)
{
 // Return a persistent name for map colour IDs; distinguish read errors from unknown colours.
 static const char *const names[]={"unknown","black","blue","green","yellow","red","white"};
 if (colour==-1) return "read error";
 if (colour<0||colour>6) return "unknown";
 return names[colour];
}

int beep(void)
{
 int tones[50][3]={{880,200,25},{-1,-1,-1}};
 return BT_play_tone_sequence(tones);
}

int find_street(void)   
{
 /*
  * This function gets your robot onto a street, wherever it is placed on the map. You can do this in many ways, but think
  * about what is the most effective and reliable way to detect a street and stop your robot once it's on it.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function
  */   
  return(0);
}

int drive_along_street(void)
{
 /*
  * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
  * the map. It stops at an intersection.
  * 
  * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
  * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
  * or the course instructor for help carrying out your plan.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function.
  */   
  return(0);
}

int scan_intersection(int *tl, int *tr, int *br, int *bl)
{
 /*
  * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
  * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
  * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
  * it needs to read, repeatably, and as the robot moves over the map.
  * 
  * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and 
  * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
  * 
  * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
  * in the code below so your TA can quickly understand how it works.
  * 
  * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
  * variables:
  * 
  * tl - top left building colour
  * tr - top right building colour
  * br - bottom right building colour
  * bl - bottom left building colour
  * 
  * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
  * after this call.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
 *(tl)=-1;
 *(tr)=-1;
 *(br)=-1;
 *(bl)=-1;
 return(0);
 
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
  return(0);
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

 // Return an invalid location/direction and notify that localization was unsuccessful (you will delete this and replace it
 // with your code).
 *(robot_x)=-1;
 *(robot_y)=-1;
 *(direction)=-1;
 return(0);
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

static double colour_means[6][3];             // Ambient-corrected RGB references, cached after loading.
static double colour_radius_squared[6];      // Squared RGB distance allowed around each reference.
static const int colour_ids[6]={5,3,2,4,1,6}; // Capture order mapped to the map's colour IDs.
static int colour_calibration_loaded=0;      // References are usable only after all six files validate.
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
 // Load six 15-sample files once, cache corrected RGB means and rejection radii; return 0 or -1 and reset filter history.
 const char *colours[]={"red","green","blue","yellow","black","white"};
 // Build locally and disable classification until the entire replacement model is valid.
 double means[6][3]={0};
 colour_calibration_loaded=0;
 reset_sensor_color_filter();
 for (int colour_index=0;colour_index<6;colour_index++)
 {
  char filename[1024],header[128];
  int length=snprintf(filename,sizeof(filename),"%s/%s.txt",directory,colours[colour_index]);
  if (length<0||(size_t)length>=sizeof(filename)) return -1;
  FILE *input=fopen(filename,"r");
  if (input==NULL) return -1;
  if (fgets(header,sizeof(header),input)==NULL||strcmp(header,"sample R G B ambient\n")!=0)
  {
   fclose(input);
   return -1;
  }
  // Require all 15 numbered rows; incomplete captures must not silently bias the colour reference.
  int failed=0;
  for (int sample=1;sample<=15;sample++)
  {
   int index,red,green,blue,ambient;
   if (fscanf(input,"%d %d %d %d %d",&index,&red,&green,&blue,&ambient)!=5||index!=sample||
     red<0||red>32767||green<0||green>32767||blue<0||blue>32767||ambient<0||ambient>32767)
   {
  failed=1;
  break;
   }
  // Preserve absolute brightness to distinguish black from white; only subtract measured ambient light.
  means[colour_index][0]+=(red-ambient)/15.0;
   means[colour_index][1]+=(green-ambient)/15.0;
   means[colour_index][2]+=(blue-ambient)/15.0;
  }
  // Allow trailing whitespace, but reject extra rows or text from an incompatible file format.
  int character;
  while (!failed&&(character=fgetc(input))!=EOF)
   if (!isspace((unsigned char)character)) failed=1;
  if (ferror(input)) failed=1;
  fclose(input);
  if (failed) return -1;
 }
 // Derive each acceptance radius from its closest competing colour, not the capture's narrow min/max range.
 for (int colour_index=0;colour_index<6;colour_index++)
 {
  double nearest_squared=1e30;
  for (int other=0;other<6;other++)
  {
   if (other==colour_index) continue;
   double distance_squared=0;
   for (int channel=0;channel<3;channel++)
   {
  double difference=means[colour_index][channel]-means[other][channel];
  distance_squared+=difference*difference;
   }
   if (distance_squared<nearest_squared) nearest_squared=distance_squared;
  }
  // Nearly identical references cannot reliably distinguish their labels.
  if (nearest_squared<1) return -1;
  // Radius is 40% of nearest-reference distance (0.4 squared); validate this heuristic with independent captures.
  colour_radius_squared[colour_index]=0.16*nearest_squared;
 }
 memcpy(colour_means,means,sizeof(means));
 colour_calibration_loaded=1;
 return 0;
}

static int classify_sensor_rgb(const int channels[4])
{
 // Compare one ambient-corrected sample to cached references; this function performs no file or sensor I/O.
 if (!colour_calibration_loaded) return 0;
 int closest=-1;
 double best_squared=1e30;
 for (int colour_index=0;colour_index<6;colour_index++)
 {
  // Squared Euclidean distance gives the same nearest mean without computing square roots.
  double distance_squared=0;
  for (int channel=0;channel<3;channel++)
  {
   double difference=channels[channel]-channels[3]-colour_means[colour_index][channel];
   distance_squared+=difference*difference;
  }
  if (distance_squared<best_squared)
  {
   best_squared=distance_squared;
   closest=colour_index;
  }
 }
 // Even an unrelated surface has a nearest reference; reject it unless it is close enough.
 return best_squared<=colour_radius_squared[closest] ? colour_ids[closest] : 0;
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
 // Requires loaded calibration and an open socket; one fresh sample per call returns confirmed ID, initially 0, or -1 on failure.
 if (!colour_calibration_loaded) return -1;
 struct timeval timeout={5,0};
 if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
   setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0)
  return update_sensor_color_filter(-1);
 // Type 4/mode 5 returns packed 16-bit RGBA within the 12-byte raw reply allocation.
 unsigned char operations[]={opINPUT_DEVICE,READY_RAW,0,COLOUR_SENSOR_PORT,4,5,3,0x60,0x64,0x68};
 unsigned char payload[12];
 if (calibration_command(operations,sizeof(operations),payload,sizeof(payload))!=0)
  return update_sensor_color_filter(-1);
 int channels[4];
 // Negative signed channel values indicate invalid data, not a colour transition worth confirming.
 for (int channel=0;channel<4;channel++)
 {
  int value=payload[2*channel]+256*payload[2*channel+1];
  if (value>=32768) return update_sensor_color_filter(0);
  channels[channel]=value;
 }
 return update_sensor_color_filter(classify_sensor_rgb(channels));
}

int calibrate_sensor(void)
{
 // On an open NXT connection, prompt for six colours and save 15 unprocessed RGBA samples each to a unique run; return 0 or -1.
 const char *colours[]={"red","green","blue","yellow","black","white"};
 struct timeval timeout={5,0};
 if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
     setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0)
 {
  perror("Unable to set calibration communication timeout");
  return -1;
 }
 unsigned char identify[]={opINPUT_DEVICE,GET_TYPEMODE,0,COLOUR_SENSOR_PORT,0x60,0x61};
 unsigned char device[2];
 if (calibration_command(identify,sizeof(identify),device,2)!=0||device[0]!=4)
 {
  fprintf(stderr,"Calibration requires an NXT colour sensor on input %d.\n",COLOUR_SENSOR_PORT+1);
  return -1;
 }
 if (mkdir("calibration",0755)!=0&&errno!=EEXIST)
 {
  perror("Unable to create calibration directory");
  return -1;
 }
 // A unique run preserves earlier captures and keeps partial/failed sessions separate from complete ones.
 char directory[]="calibration/run-XXXXXX";
 if (mkdtemp(directory)==NULL)
 {
  perror("Unable to create calibration session");
  return -1;
 }
 printf("Saving raw calibration readings in %s\n",directory);
 for (size_t colour_index=0;colour_index<6;colour_index++)
 {
  printf("Now reading %s colour: please move the colour sensor over %s and press Enter in this terminal.\n",
    colours[colour_index],colours[colour_index]);
  fflush(stdout);
  if (wait_for_calibration_enter()!=0)
  {
   fprintf(stderr,"Terminal input closed; calibration stopped.\n");
   return -1;
  }
  char filename[256];
  snprintf(filename,sizeof(filename),"%s/%s.txt",directory,colours[colour_index]);
  FILE *output=fopen(filename,"wx");
  if (output==NULL) {perror("Unable to create colour file");return -1;}
  int failed=fprintf(output,"sample R G B ambient\n")<0;
  // Store uncorrected channels so later processing can change without requiring a new hardware capture.
  for (int sample=1;sample<=15&&!failed;sample++)
  {
   unsigned char operations[]={opINPUT_DEVICE,READY_RAW,0,COLOUR_SENSOR_PORT,4,5,3,0x60,0x64,0x68};
   unsigned char payload[12];
   if (calibration_command(operations,sizeof(operations),payload,sizeof(payload))!=0)
   {
    fprintf(stderr,"Sensor read failed; %s is incomplete.\n",filename);
    failed=1;
    break;
   }
   int channels[4];
   // NXT mode 5 packs four 16-bit channels into the raw payload.
   for (int channel=0;channel<4;channel++)
   {
    int value=payload[2*channel]+256*payload[2*channel+1];
    channels[channel]=value>=32768 ? value-65536 : value;
   }
   failed=fprintf(output,"%d %d %d %d %d\n",sample,channels[0],channels[1],channels[2],channels[3])<0;
   if (fflush(output)!=0) failed=1;
   printf("%s %d/15: R=%d G=%d B=%d ambient=%d\n",colours[colour_index],sample,
     channels[0],channels[1],channels[2],channels[3]);
   if (sample<15) usleep(100000);
  }
  // Signal completion only after all rows have been written and the file has closed successfully.
  if (fclose(output)!=0) failed=1;
  if (failed) {fprintf(stderr,"Calibration stopped; check %s.\n",filename);return -1;}
  if (beep()!=0) {fprintf(stderr,"Completion beep failed.\n");return -1;}
  printf("Saved 15 readings to %s\n",filename);
 }
 printf("Calibration capture complete. No averaging or colour classification applied.\n");
 return 0;
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
     G=*(map_img+((k+(by*rx))*3)+1);
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