#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#define PORT_1 0
#define PORT_4 3
#define opINPUT_DEVICE 0x99
#define READY_RAW 0x1c
static int descriptor=7;
static int *socket_id=&descriptor;
static int message_id_counter=1,ref_angle=3;
static unsigned char incoming[13],outgoing[17];
static size_t received,sent,incoming_size,chunk_size;
static int interrupt_read,interrupt_write,fail_read,fail_write,zero_write,cases;
static ssize_t fixture_write(int fd,const void *buffer,size_t size)
{
 assert(fd==descriptor);
 if (interrupt_write) {interrupt_write=0;errno=EINTR;return -1;}
 if (fail_write) {errno=EIO;return -1;}
 if (zero_write) return 0;
 if (size>chunk_size) size=chunk_size;
 assert(sent+size<=sizeof(outgoing));
 memcpy(outgoing+sent,buffer,size);sent+=size;return (ssize_t)size;
}
static ssize_t fixture_read(int fd,void *buffer,size_t size)
{
 assert(fd==descriptor);
 if (interrupt_read) {interrupt_read=0;errno=EINTR;return -1;}
 if (fail_read) {errno=EIO;return -1;}
 if (size>chunk_size) size=chunk_size;
 if (size>incoming_size-received) size=incoming_size-received;
 memcpy(buffer,incoming+received,size);received+=size;return (ssize_t)size;
}
#define write fixture_write
#define read fixture_read
#include "gyro_reader_under_test.inc"
#undef write
#undef read
#define CHECK(expression) do {assert(expression);cases++;} while (0)
static void encode(unsigned char *destination,int32_t value)
{
 uint32_t bits=(uint32_t)value;
 for (int byte=0;byte<4;byte++) destination[byte]=(unsigned char)(bits>>(8*byte));
}
static void reset(int32_t angle,int32_t rate)
{
 memset(incoming,0,sizeof(incoming));memset(outgoing,0,sizeof(outgoing));
 incoming[0]=11;incoming[2]=1;incoming[4]=2;
 encode(incoming+5,angle);encode(incoming+9,rate);
 received=sent=0;incoming_size=sizeof(incoming);chunk_size=sizeof(outgoing);
 interrupt_read=interrupt_write=fail_read=fail_write=zero_write=0;
 message_id_counter=1;ref_angle=3;socket_id=&descriptor;
}
static int rejected(int reset_reference)
{
 int angle=12345,rate=6789,old_reference=ref_angle;
 int result=BT_read_gyro(1,reset_reference,&angle,&rate);
 return result==-1&&angle==12345&&rate==6789&&ref_angle==old_reference;
}
int main(void)
{
 int angle,rate;
 reset(100,5);CHECK(BT_read_gyro(1,0,&angle,&rate)==1&&angle==97&&rate==5&&ref_angle==3);
 CHECK(sent==17&&received==13&&outgoing[5]==8&&outgoing[10]==1&&message_id_counter==2);
 reset(-15,-5);CHECK(BT_read_gyro(1,1,&angle,&rate)==1&&angle==0&&rate==-5&&ref_angle==-15);
 reset(INT32_MIN,0);CHECK(rejected(0)); /* The observed sentinel/reference overflow. */
 reset(INT32_MIN,0);CHECK(rejected(1));
 reset(10,INT32_MIN);CHECK(rejected(1));
 reset(-32768,0);CHECK(rejected(1));
 reset(10,-32768);CHECK(rejected(0));
 reset(INT32_MAX,0);ref_angle=-2;CHECK(rejected(0));
 reset(INT32_MIN+1,0);CHECK(rejected(0));
 reset(-15,0);CHECK(BT_read_gyro(1,0,&angle,&rate)==1&&angle==-18);
 reset(100,5);chunk_size=1;
 CHECK(BT_read_gyro(1,0,&angle,&rate)==1&&angle==97&&sent==17&&received==13);
 reset(100,5);interrupt_read=interrupt_write=1;
 CHECK(BT_read_gyro(1,0,&angle,&rate)==1&&angle==97);
 reset(100,5);incoming_size=8;CHECK(rejected(1));
 reset(100,5);incoming[0]=10;CHECK(rejected(1));
 reset(100,5);incoming[2]=2;CHECK(rejected(1));
 reset(100,5);incoming[4]=4;CHECK(rejected(1));
 reset(100,5);zero_write=1;CHECK(rejected(1));
 reset(100,5);fail_write=1;CHECK(rejected(1));
 reset(100,5);fail_read=1;CHECK(rejected(1));
 reset(100,5);CHECK(BT_read_gyro(4,0,&angle,&rate)==-1&&sent==0);
 reset(100,5);CHECK(BT_read_gyro((char)-1,0,&angle,&rate)==-1&&sent==0);
 reset(100,5);CHECK(BT_read_gyro(1,0,NULL,&rate)==-1&&sent==0);
 reset(100,5);CHECK(BT_read_gyro(1,0,&angle,NULL)==-1&&sent==0);
 reset(100,5);socket_id=NULL;CHECK(rejected(1)&&sent==0);
 reset(100,5);message_id_counter=65535;incoming[2]=incoming[3]=255;
 CHECK(BT_read_gyro(1,0,&angle,&rate)==1&&outgoing[2]==255&&outgoing[3]==255&&angle==97);
 printf("%d/%d gyro framing, invalid-data, overflow and unchanged-reference cases passed with UBSan.\n",cases,cases);
 return 0;
}
