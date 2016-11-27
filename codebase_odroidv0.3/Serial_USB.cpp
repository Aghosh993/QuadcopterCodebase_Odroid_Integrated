#include "Serial_USB.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

#include "quadrotor_comm.h"


static Serial_USB *serial_usb = 0;

static int serial_port_fd;
static void setup_linux_serial(char* serialPort_name)
{
  struct termios tio;
  int tty_fd;

  memset(&tio,0,sizeof(tio));
  tio.c_iflag=0;
  tio.c_oflag=0;
  tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
  tio.c_lflag=0;
  tio.c_cc[VMIN]=1;
  tio.c_cc[VTIME]=5;

  tty_fd=open(serialPort_name, O_RDWR);// | O_BLOCK);     
    
  // No baud rate needed for USB CDC ACM interface... uncomment and adjust adjust accordingly if using a real UART interface:
    
    
  cfsetospeed(&tio,B115200);            // 57600 baud
  cfsetispeed(&tio,B115200);            // 57600 baud
    

  tcsetattr(tty_fd,TCSANOW,&tio);

  serial_port_fd = tty_fd;
}

Serial_USB::Serial_USB()
{
  setup_linux_serial("/dev/ttyUSB0");
}

void Serial_USB::send_bytes(uint8_t *b, int n)
{
  write(serial_port_fd, b, n);
}

int Serial_USB::read_some_bytes(uint8_t *b, int n)
{
  return read(serial_port_fd, b, n);
}


void init_serial_usb_comm()
{
  serial_usb = new Serial_USB();
}

void send_bytes(uint8_t *b, int n)
{
  if (serial_usb != 0) serial_usb->send_bytes(b, n);
}

void update_serial_usb_comm()
{
  while (1)
  {
    uint8_t b[16];
    int r = serial_usb->read_some_bytes(b, 16);
    if (r > 0)recv_bytes(b, r);
    else return;
  }
}
