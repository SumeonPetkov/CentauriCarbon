
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include<termios.h>
#include "SharespaceInit.h"
#include "debug.h"


static int fd_uart;

extern struct DspMemOps R528_Dsp_Uart_Ops ;
int init_dsp_uart()
{
   int ret = 0;
   printf("init_dsp_uart\n");
    int fd = sharespace_mmap();
     printf("sharespace_mmap %d\n",fd );
     if(fd < 0)
     {
          return -1;
     }
     char *dev = "/dev/ttyS2"; //串口二
     fd_uart = open(dev,O_RDWR  | O_SYNC  | O_NOCTTY | O_NONBLOCK);
    if (fd_uart < 0)
    {
         GAM_DEBUG_printf("open /dev/ttyS1 failed!\n");
          return -1;
    }
    if (fcntl(fd_uart, F_SETFL, 0) < 0)//设置文件状态标记 
    {
        GAM_DEBUG_printf("fcntl failed!\n");
    }
    else
    {
        GAM_DEBUG_printf("fcntl=%d\n", fcntl(fd_uart, F_SETFL, 0));
    }
    /*测试是否为终端设备*/
    if (isatty(STDIN_FILENO) == 0)
    {
        GAM_DEBUG_printf("standard input is not a terminal device\n");
    }
    else
    {
        GAM_DEBUG_printf("isatty success!\n");
    }
     struct termios oldtio;   
     if( tcgetattr( fd_uart,&oldtio)  !=  0) { //保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息
          printf("SetupSerial ttyS1");  
          printf("tcgetattr( fd_uart,&oldtio) -> %d\n",tcgetattr( fd_uart,&oldtio));   
          return -1;   
     }
    oldtio.c_cflag &= ~CSIZE;    //clear数据位 
    oldtio.c_cflag |= CS8;      //set数据位 8
    oldtio.c_cflag &= ~CSTOPB;//设置停止位 1
    oldtio.c_cflag |= CREAD | CLOCAL;   //设置字符大小
    oldtio.c_cflag &= ~PARENB; //校验位 无奇偶校验位
    oldtio.c_cflag &= ~CRTSCTS; //流控 NONE

   oldtio.c_lflag &= ~ICANON; //禁用规范模式
    oldtio.c_lflag &= ~ECHO;   // Disable echo
    oldtio.c_lflag &= ~ECHOE;  // Disable erasure
    oldtio.c_lflag &= ~ECHONL; // Disable new-line echo
    oldtio.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    oldtio.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    oldtio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
    oldtio.c_oflag &= ~OPOST;                                                       // Prevent special interpretation of output bytes (e.g. newline chars)
    oldtio.c_oflag &= ~ONLCR;                                                       // Prevent conversion of newline to carriage return/line feed

    // oldtio.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // oldtio.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    oldtio.c_cc[VTIME] = 0;   //设置等待时间和最小接收字符
    oldtio.c_cc[VMIN] = 0;

 
     // cfsetispeed(&oldtio, B115200);   //设置输入的波特率
     // cfsetospeed(&oldtio, B115200); //设置输出的波特率
     cfsetspeed(&oldtio, B115200);
 
     tcflush(fd_uart,TCIFLUSH);   //处理未接收字符
     
     if((tcsetattr(fd_uart,TCSANOW,&oldtio))!=0)   //激活新配置
     {   
          perror("com set error");   
          return -1;   
     }   
     else{
          printf("set done!\n");   
     } 
     ioctl(fd_uart, TCFLSH, 2);
     // char *uartcs_str = "UABA\n"; //串口二
     //      write(fd_uart, (void *)uartcs_str,5);


     sharespace_wait_dsp_init();
    R528_Dsp_Uart_Ops.fd_write = fd_uart;
    R528_Dsp_Uart_Ops.fd_read = fd;
   R528_Dsp_Uart_Ops.write_lenth = 0;
    R528_Dsp_Uart_Ops.read_lenth = 0;
     printf("init_dsp_uart OK\n");
     return 0;
}

int deinit_dsp_uart()
{
     return 0;
}

int read_dsp_uart(uint8_t* buf)
{
      return sharespace_read_dsp(buf);
}

void set_read_dsp_uart_pos(uint32_t read_addr)
{
     // set_read_dsp_space_pos();
}

int write_dsp_uart(uint8_t* cmd, int len)   //-6-send-g-g-2022-06-16
{
     return write(fd_uart, (void *)cmd, len);
}

struct DspMemOps R528_Dsp_Uart_Ops =
{
    init:				init_dsp_uart,
    de_init:				deinit_dsp_uart,
    mem_read:			read_dsp_uart,
    set_read_pos:			set_read_dsp_uart_pos,
    fd_write_type:			SQT_UART,
    mem_write:			write_dsp_uart
};

struct DspMemOps* GetDspUartOps()
{
	return &R528_Dsp_Uart_Ops;
}


