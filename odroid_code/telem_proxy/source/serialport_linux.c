#include "serialport_linux.h"

static int serial_port_fd;

void setup_linux_serial(char* serialPort_name)
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
	
	
    cfsetospeed(&tio,B460800);
    cfsetispeed(&tio,B460800);
	

    tcsetattr(tty_fd,TCSANOW,&tio);

    serial_port_fd = tty_fd;
}

void uart_send_byte_linux(uint8_t xmit_byte)
{
	write(serial_port_fd, &xmit_byte, 1);
}

uint8_t recv_byte()
{
    int bytes_read = 0;
    char read_char;
    while(bytes_read < 1)
    {
        bytes_read = read(serial_port_fd, &read_char, 1);
    }
    return read_char;
}

void recv_data(uint8_t *data_buf, uint8_t len)
{
    int bytes_to_read = len;
    int bytes_read = 0;
    while(bytes_to_read > 0)
    {
        bytes_read = read(serial_port_fd, &data_buf[len-bytes_to_read], 1);
        bytes_to_read -= bytes_read;
    }
}

int get_serial_fd(void)
{
	return serial_port_fd;
}