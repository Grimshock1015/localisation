/* Compile the actual map/PPM parser without any Bluetooth dependencies. */
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static struct {
 unsigned int before;
 int values[400][4];
 unsigned int after;
} storage;
#define map storage.values
int sx,sy;

#include "map_parser_under_test.inc"

int main(int argc,char **argv)
{
 storage.before=0x12345678u;storage.after=0x87654321u;
 for (int i=0;i<400;i++) for (int c=0;c<4;c++) map[i][c]=12345;
 int original[400][4];memcpy(original,map,sizeof(map));
 sx=sy=0;
 if (argc==2&&strcmp(argv[1],"invalid_arguments")==0)
 {
  unsigned char pixel[3]={0};int x=5,y=5;
  int ok=!parse_map(NULL,2,2)&&!parse_map(pixel,0,1)&&!parse_map(pixel,-1,1)&&
         !parse_map(pixel,1,0)&&!parse_map(pixel,1,-1)&&
         readPPMimage(NULL,&x,&y)==NULL&&x==0&&y==0&&
         readPPMimage("unused",NULL,&y)==NULL&&readPPMimage("unused",&x,NULL)==NULL&&
         readPPMimage("unused",&x,&x)==NULL;
  printf("{\"arguments_ok\":%d,\"unchanged\":%d}\n",ok,memcmp(original,map,sizeof(map))==0);
  return 0;
 }
 if (argc!=3) return 2;
 int width=-1,height=-1;
 unsigned char *image=readPPMimage(argv[2],&width,&height);
 int parsed=image!=NULL&&strcmp(argv[1],"parse")==0 ? parse_map(image,width,height) : 0;
 printf("{\"read\":%d,\"width\":%d,\"height\":%d,\"parsed\":%d,\"sx\":%d,\"sy\":%d,",
        image!=NULL,width,height,parsed,sx,sy);
 printf("\"canaries\":%d,\"unchanged\":%d,\"prefix\":[",
        storage.before==0x12345678u&&storage.after==0x87654321u,memcmp(original,map,sizeof(map))==0);
 if (image!=NULL) for (size_t i=0;i<6&&i<(size_t)width*(size_t)height*3;i++)
  printf("%s%d",i ? "," : "",image[i]);
 printf("],\"map\":[");
 if (parsed) for (int i=0;i<sx*sy;i++)
  printf("%s[%d,%d,%d,%d]",i ? "," : "",map[i][0],map[i][1],map[i][2],map[i][3]);
 printf("]}\n");
 free(image);
 return 0;
}
