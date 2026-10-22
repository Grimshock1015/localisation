/* Hardware substitutes for the production localization functions.
 * Compiled by simulate_localization.py; no Bluetooth code is linked.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "simulation_map.inc"

int map[400][4];
int sx,sy;
double beliefs[400][4];
static int actual_x,actual_y,actual_heading;
static int scans,trips,boundaries;
static double wrong_probability,unknown_probability;
static double turn_error_probability,stay_probability;
static int scan_offset,trace_enabled;
static uint32_t random_state;

// Avoid writing large traces during sweeps; keep CSV output and assertion errors.
static int simulation_printf(const char *format,...)
{
 if (!trace_enabled) return 0;
 va_list args;va_start(args,format);
 int result=vprintf(format,args);va_end(args);return result;
}

static int simulation_fprintf(FILE *stream,const char *format,...)
{
 if (stream==stderr&&!trace_enabled) return 0;
 va_list args;va_start(args,format);
 int result=vfprintf(stream,format,args);va_end(args);return result;
}

#define printf simulation_printf
#define fprintf simulation_fprintf

static double random_unit(void)
{
 random_state=1664525u*random_state+1013904223u;
 return random_state/4294967296.0;
}

static void check_beliefs(void)
{
 double total=0.0;
 for (int i=0;i<sx*sy;i++)
  for (int h=0;h<4;h++)
  {
   assert(isfinite(beliefs[i][h])&&beliefs[i][h]>0.0);
   total+=beliefs[i][h];
  }
 assert(fabs(total-1.0)<1e-10);
}

static int find_street(void) {return 1;}
static int align_to_street(void) {return 1;}

static int drive_to_next_intersection(int *direction,int *boundary_return)
{
 check_beliefs();
 *boundary_return=0;
 // Start poses describe the first scanned intersection. The initial approach
 // arrives there without an intersection-to-intersection motion update.
 if (trips++==0) return 1;
 const int dx[4]={0,1,0,-1},dy[4]={-1,0,1,0};
 int x=actual_x+dx[actual_heading],y=actual_y+dy[actual_heading];
 if (x<0||x>=sx||y<0||y>=sy)
 {
  *boundary_return=1;
  boundaries++;
  actual_heading=(actual_heading+2)%4;
  if (turn_error_probability>0.0&&random_unit()<turn_error_probability)
   actual_heading=(actual_heading+(random_unit()<0.5 ? 1 : 3))%4;
  *direction=(*direction+2)%4;
 }
 else if (!(stay_probability>0.0&&random_unit()<stay_probability))
 {actual_x=x;actual_y=y;}
 return 1;
}

static int turn_at_intersection(int turn_direction)
{
 assert(turn_direction==0||turn_direction==1);
 int shift=turn_direction==0 ? 1 : 3;
 if (turn_error_probability>0.0&&random_unit()<turn_error_probability)
  shift=random_unit()<0.5 ? 0 : 2;
 actual_heading=(actual_heading+shift)%4;
 return 1;
}

static int scan_intersection(int *tl,int *tr,int *br,int *bl)
{
 check_beliefs();
 scans++;
 int *outputs[4]={tl,tr,br,bl};
 const int colour_ids[3]={2,3,6};
 int complete=1;
 printf("SIM truth: (%d,%d), heading=%d, scan=%d\n",
        actual_x,actual_y,actual_heading,scans);
 for (int c=0;c<4;c++)
 {
  int colour=map[actual_y*sx+actual_x][(c+actual_heading+scan_offset)%4];
  // Independent draws: wrong_probability applies to non-missing readings.
  if (random_unit()<unknown_probability) {colour=-1;complete=0;}
  else if (random_unit()<wrong_probability)
  {
   int alternatives[2],count=0;
   for (int k=0;k<3;k++) if (colour_ids[k]!=colour) alternatives[count++]=colour_ids[k];
   colour=alternatives[random_unit()<0.5 ? 0 : 1];
  }
  *outputs[c]=colour;
 }
 return complete;
}

static const char *get_colour_name(int colour)
{
 switch (colour)
 {
  case 2:return "blue";
  case 3:return "green";
  case 6:return "white";
  default:return "unknown";
 }
}

// Inserted verbatim from the current EV3_Localization.c at test time.
#include "localization_under_test.inc"

static void reset_case(int x,int y,int heading,uint32_t seed)
{
 actual_x=x;actual_y=y;actual_heading=heading;
 scans=trips=boundaries=0;
 random_state=seed;
 for (int i=0;i<sx*sy;i++)
  for (int h=0;h<4;h++) beliefs[i][h]=1.0/(sx*sy*4);
}

static void check_corner_order(void)
{
 const int corners[4]={2,3,6,2};
 const int observations[4][4]={{2,3,6,2},{3,6,2,2},{6,2,2,3},{2,2,3,6}};
 for (int h=0;h<4;h++)
 {
  double correct=score_intersection_scan(corners,observations[h],h);
  assert(fabs(correct-0.4096)<1e-12);
  for (int other=0;other<4;other++)
   if (other!=h) assert(correct>score_intersection_scan(corners,observations[h],other));
 }
}

static void check_motion_models(void)
{
 sx=sy=3;
 const int next[4]={1,5,7,3};
 const double floor=1e-6/36.0;
 for (int h=0;h<4;h++)
 {
  for (int turn=0;turn<2;turn++)
  {
   memset(beliefs,0,sizeof(beliefs));beliefs[4][h]=1;
   assert(update_beliefs_after_turn(turn));check_beliefs();
   int intended=(h+(turn==0 ? 1 : 3))%4;
   assert(fabs(beliefs[4][intended]-(0.96*(1-1e-6)+floor))<1e-12);
   assert(fabs(beliefs[0][0]-floor)<1e-12); // No cell displacement.
  }
  memset(beliefs,0,sizeof(beliefs));beliefs[4][h]=1;
  assert(update_beliefs_after_forward(0));check_beliefs();
  assert(fabs(beliefs[next[h]][h]-(0.94*(1-1e-6)+floor))<1e-12);
  assert(fabs(beliefs[4][h]-(0.06*(1-1e-6)+floor))<1e-12);
  memset(beliefs,0,sizeof(beliefs));beliefs[next[h]][h]=1;
  assert(update_beliefs_after_forward(1));check_beliefs();
  assert(fabs(beliefs[next[h]][(h+2)%4]-(0.96*(1-1e-6)+floor))<1e-12);
 }
 double before[400][4];memcpy(before,beliefs,sizeof(before));
 assert(!update_beliefs_after_forward(2));assert(!update_beliefs_after_turn(2));
 assert(memcmp(before,beliefs,sizeof(before))==0);
 // A logically impossible trip must fail without corrupting the prior.
 sx=sy=1;memset(beliefs,0,sizeof(beliefs));beliefs[0][0]=1;
 memcpy(before,beliefs,sizeof(before));
 assert(!update_beliefs_after_forward(0));
 assert(memcmp(before,beliefs,sizeof(before))==0);
 sx=400;sy=1;reset_case(0,0,0,1);
 for (int i=0;i<400;i++)
  for (int c=0;c<4;c++) map[i][c]=(int[]){2,3,6,2}[c];
 const int observed[4]={2,3,6,2},unknown[4]={-1,0,4,5};
 for (int n=0;n<100;n++) {assert(update_beliefs_from_scan(observed));check_beliefs();}
 assert(update_beliefs_from_scan(unknown));check_beliefs();
 sx=401;assert(!update_beliefs_after_forward(0));assert(!update_beliefs_after_turn(0));
 assert(!update_beliefs_from_scan(observed));
}

static void check_ambiguity_and_missing(void)
{
 sx=sy=2;
 for (int i=0;i<4;i++) for (int c=0;c<4;c++) map[i][c]=2;
 wrong_probability=unknown_probability=0.0;
 reset_case(0,0,0,1);
 int x,y,h;
 assert(!robot_localization(&x,&y,&h));
 assert(scans==24);check_beliefs();
 // All-unknown scans on the real map must not count as confirmation either.
 sx=SIM_WIDTH;sy=SIM_HEIGHT;
 memcpy(map,simulation_map,sizeof(simulation_map));
 unknown_probability=1.0;
 reset_case(0,0,0,1);
 assert(!robot_localization(&x,&y,&h));
 assert(scans==24);check_beliefs();
}

int main(int argc,char **argv)
{
 // Keep stdout and stderr in execution order when the runner captures a trace.
 setvbuf(stdout,NULL,_IONBF,0);
 setvbuf(stderr,NULL,_IONBF,0);
 // The Python runner validates all arguments before calling this executable.
 assert(argc==13);
 check_corner_order();
 check_motion_models();
 check_ambiguity_and_missing();
 wrong_probability=atof(argv[1]);unknown_probability=atof(argv[2]);
 int trials=atoi(argv[3]);uint32_t seed=(uint32_t)strtoul(argv[4],NULL,10);
 int start_x=atoi(argv[5]),start_y=atoi(argv[6]),start_h=atoi(argv[7]);
 FILE *report=fopen(argv[8],"w");assert(report!=NULL);
 int trace=atoi(argv[9]);
 trace_enabled=trace;
 turn_error_probability=atof(argv[10]);stay_probability=atof(argv[11]);scan_offset=atoi(argv[12]);
 fprintf(report,"trial,seed,start_x,start_y,start_heading,result,scans,boundary_returns,true_x,true_y,true_heading,estimated_x,estimated_y,estimated_heading\n");
 int failures=0,case_number=0;
 for (int trial=0;trial<trials;trial++)
  for (int y=0;y<sy;y++) for (int x=0;x<sx;x++) for (int h=0;h<4;h++)
  {
   if (start_x>=0&&(x!=start_x||y!=start_y||h!=start_h)) continue;
   uint32_t case_seed=seed+(uint32_t)case_number++;
   reset_case(x,y,h,case_seed);
   printf("SIM case: trial=%d start=(%d,%d,%d)\n",trial,x,y,h);
   int px=-1,py=-1,ph=-1;
   int success=robot_localization(&px,&py,&ph);
   check_beliefs();
   const char *result=!success ? "unresolved" :
     px==actual_x&&py==actual_y&&ph==actual_heading ? "correct" : "incorrect";
   if (strcmp(result,"correct")!=0) failures++;
   fprintf(report,"%d,%lu,%d,%d,%d,%s,%d,%d,%d,%d,%d,%d,%d,%d\n",
     trial,(unsigned long)case_seed,x,y,h,result,scans,boundaries,actual_x,actual_y,actual_heading,px,py,ph);
   if (trace) printf("SIM result: %s; truth=(%d,%d,%d), estimate=(%d,%d,%d)\n",
     result,actual_x,actual_y,actual_heading,px,py,ph);
  }
 assert(fclose(report)==0);
 return failures ? 1 : 0;
}
