#include "../EV3_Localization.h"
#include <signal.h>
#include <sys/wait.h>
#include <assert.h>

extern int *socket_id;
static volatile sig_atomic_t trial_socket=-1;
static const unsigned char trial_brake_packet[]={
 0x09,0x00,0xfe,0xff,0x80,0x00,0x00,0xa3,0x00,MOTOR_B|MOTOR_C,0x01
};

static void trial_stop_signal(int signal_number)
{
 // No library allocation or buffered IO in the handler. Dispatch both brakes
 // before closing the existing connection; external reconnects remain a fallback.
 int descriptor=trial_socket;
 if (descriptor>=0)
 {
  ssize_t sent=send(descriptor,trial_brake_packet,sizeof(trial_brake_packet),
                    MSG_NOSIGNAL|MSG_DONTWAIT);
  if (sent==(ssize_t)sizeof(trial_brake_packet))
  {
   const char message[]="\nTrial signal: B/C brake packet sent on existing connection.\n";
   write(STDERR_FILENO,message,sizeof(message)-1);
  }
  else
  {
   const char message[]="\nTrial signal: brake dispatch failed; check physical standstill.\n";
   write(STDERR_FILENO,message,sizeof(message)-1);
  }
 }
 _exit(128+signal_number);
}

static int trial_open(const char *address)
{
 int result=BT_open(address);
 if (result==0) trial_socket=*socket_id;
 return result;
}

static int trial_close(void)
{
 if (trial_socket>=0)
  send(trial_socket,trial_brake_packet,sizeof(trial_brake_packet),MSG_NOSIGNAL|MSG_DONTWAIT);
 trial_socket=-1;
 return BT_close();
}

#define BT_open trial_open
#define BT_close trial_close
#define main localization_application_main
#include "../EV3_Localization.c"
#undef main
#undef BT_open
#undef BT_close

static int check_signal_brake(void)
{
 int pair[2];
 assert(socketpair(AF_UNIX,SOCK_STREAM,0,pair)==0);
 pid_t child=fork();
 assert(child>=0);
 if (child==0)
 {
  close(pair[0]);
  trial_socket=pair[1];
  trial_stop_signal(SIGTERM);
 }
 close(pair[1]);
 unsigned char packet[sizeof(trial_brake_packet)];
 assert(recv(pair[0],packet,sizeof(packet),MSG_WAITALL)==(ssize_t)sizeof(packet));
 assert(memcmp(packet,trial_brake_packet,sizeof(packet))==0);
 assert(packet[9]==6 && packet[10]==1 && packet[4]==0x80);
 int status;
 assert(waitpid(child,&status,0)==child);
 assert(WIFEXITED(status) && WEXITSTATUS(status)==128+SIGTERM);
 close(pair[0]);
 puts("Signal brake dispatch self-test passed; no Bluetooth or motor connection used.");
 return 0;
}

int main(int argc,char **argv)
{
 if (argc==2 && strcmp(argv[1],"--stop-self-test")==0) return check_signal_brake();
 struct sigaction action={};
 action.sa_handler=trial_stop_signal;
 sigemptyset(&action.sa_mask);
 sigaddset(&action.sa_mask,SIGINT);
 sigaddset(&action.sa_mask,SIGTERM);
 sigaddset(&action.sa_mask,SIGHUP);
 if (sigaction(SIGINT,&action,NULL)!=0 || sigaction(SIGTERM,&action,NULL)!=0 ||
     sigaction(SIGHUP,&action,NULL)!=0) return 1;
 return localization_application_main(argc,argv);
}
