#include <time.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

#include <cstdio>
#include <cstring>
#include <iostream>

#include <ros/ros.h>
#include <vo_flow/OpticalFlow.h>

using namespace std;

int set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop);

typedef struct
{
       uint64_t  time_sec;
       unsigned char   id;
       int16_t flow_x;
       int16_t flow_y;
       float flow_comp_x;    //Flow in m in x-sensor direction, angular-speed compensated
       float flow_comp_y;
       unsigned char quality; //Optical flow quality / confidence. 0: bad, 255: maximum quality
       float height;   //ground_distance Ground distance in m. Positive value: distance known. Negative value: Unknown distance
}FLOW;

FLOW flow;
vo_flow::OpticalFlow flow_msg;
ros::Publisher flow_pub;

float ByteToFloat(unsigned char* byteArry)
{
 return *((float*)byteArry);
}


unsigned char FLOW_STATE[4];
unsigned char flow_buf[30];
void FLOW_MAVLINK(unsigned char data)
{
static unsigned char s_flow=0,data_cnt=0;
unsigned char cnt_offset=0;
unsigned char get_one_fame=0;
unsigned char floattobyte[4];
switch(s_flow)
{
   case 0: if(data==0xFE)
                s_flow=1;
                break;
   case 1: if(data==0x1A)
                { s_flow=2;}
            else
                s_flow=0;
                break;
   case 2:
            if(data_cnt<4)
                {s_flow=2; FLOW_STATE[data_cnt++]=data;}
            else
                {data_cnt=0;s_flow=3;flow_buf[data_cnt++]=data;}
                break;
   case 3:
            if(FLOW_STATE[3]==100){
                       if(data_cnt<26)
                       {s_flow=3; flow_buf[data_cnt++]=data;}
                 else
                       {data_cnt=0;s_flow=4;}
               }
               else
                       {data_cnt=0;s_flow=0;}
                        break;
   case 4:
          get_one_fame=1;
          s_flow=0;data_cnt=0;
          break;
   default:
          s_flow=0;
          data_cnt=0;
          break;
  }//--end of s_uart

    if(get_one_fame)
        {
          flow.time_sec=(flow_buf[7]<<56)|(flow_buf[6]<<48)|(flow_buf[5]<<40)|(flow_buf[4]<<32)
                 |(flow_buf[3]<<24)|(flow_buf[2]<<16)|(flow_buf[1]<<8)|(flow_buf[0]);
                floattobyte[0]=flow_buf[8];
                floattobyte[1]=flow_buf[9];
                floattobyte[2]=flow_buf[10];
                floattobyte[3]=flow_buf[11];
               flow.flow_comp_x =ByteToFloat(floattobyte);
                floattobyte[0]=flow_buf[12];
                floattobyte[1]=flow_buf[13];
                floattobyte[2]=flow_buf[14];
                floattobyte[3]=flow_buf[15];
               flow.flow_comp_y =ByteToFloat(floattobyte);
                floattobyte[0]=flow_buf[16];
                floattobyte[1]=flow_buf[17];
                floattobyte[2]=flow_buf[18];
                floattobyte[3]=flow_buf[19];
          flow.height=ByteToFloat(floattobyte);//ground_distance float Ground distance in m.
                                              //Positive value: distance known.
                                              //Negative value: Unknown distance
          flow.flow_x=(int16_t)((flow_buf[20])|(flow_buf[21]<<8));
          flow.flow_y=(int16_t)((flow_buf[22])|(flow_buf[23]<<8));
          flow.id=flow_buf[24];
          flow.quality=flow_buf[25]; //Optical flow quality / confidence. 0: bad, 255: maximum quality


          flow_msg.header.stamp = ros::Time(flow.time_sec/1000000 , (flow.time_sec%1000000) * 1000);
          //std::cout <<flow.time_sec<< std::endl;
          //flow_msg.header.stamp = ros::Time::now();
          flow_msg.header.frame_id = "px4flow";
          flow_msg.ground_distance = flow.height;
          flow_msg.flow_x = flow.flow_x;
          flow_msg.flow_y = flow.flow_y;
          flow_msg.velocity_x = flow.flow_comp_x;
          flow_msg.velocity_y = flow.flow_comp_y;
          flow_msg.quality = flow.quality;

          flow_pub.publish(flow_msg);
        }
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"serial_px4flow_node");
    ros::NodeHandle nh("~");

    flow_pub = nh.advertise<vo_flow::OpticalFlow>("/serial_flow_msg",10);
    string flow_serial_port;
    nh.param("flow_serial_port",flow_serial_port,string("/dev/ttyUSB1"));

    int i,serial_fd;
    unsigned char read_buffer;
    struct termios oldtio;
    int serialfd = open(flow_serial_port.c_str (), O_RDWR);
    if(-1 == serialfd)
      {
        perror("open serial error!");
        exit(1);
      }
    if(tcgetattr(serialfd, &oldtio) != 0)
      {
        perror("SetupSerial");
        return -1;
      }
    if((i = set_term(serialfd, 115200, 8, 'N', 1)) < 0)
      {
        perror("set_term error");
        exit(1);
      }
    int k=0;
    while(1)
      {
        struct timeval serial_timeout;
        fd_set serial_readfds;
        FD_ZERO(&serial_readfds);
        FD_SET(serialfd, &serial_readfds);
        serial_timeout.tv_sec = 0;          // 0s
        serial_timeout.tv_usec = 100000;    // 100ms
        int select_flag = select(FD_SETSIZE, &serial_readfds, (fd_set *)NULL, (fd_set *)NULL, &serial_timeout);

        switch(select_flag)
          {
            case 0:
              i++;
	          printf("serial receive error for %d times\n",i);
            break;

            case -1:
              perror("select");
              exit(1);

            default:
              for(serial_fd = 0; serial_fd < FD_SETSIZE; serial_fd++)
                {
                  if(FD_ISSET(serial_fd, &serial_readfds))
                     {
                       int data_counter = read(serial_fd, &read_buffer, 1);
                       FLOW_MAVLINK(read_buffer);
                     }
                }
            break;
          }
     }
     int closefd = close(serialfd);
}

int set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio;

    bzero(&newtio, sizeof(newtio));

    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch(nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    case 9:
        newtio.c_cflag |= CS8;
        break;
    }
    /* Set the Parity Bit */
    switch(nEvent)
    {
    case 'O': // Odd Parity
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP); // I'm not sure, but it doesn't matters
        break;
    case 'E': // Even Parity
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        newtio.c_cflag |= (INPCK | ISTRIP); // I'm not sure, but it doesn't matters
        break;
    case 'N': // None Parity
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /* Set the Baud Rate */
    switch(nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    /* Set the Stop Bit */
    if(nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if(nStop == 2)
        newtio.c_cflag |= CSTOPB;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 12;

    tcflush(fd, TCIFLUSH);

    if((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }

    printf("set done!\n");
    return 0;
}
