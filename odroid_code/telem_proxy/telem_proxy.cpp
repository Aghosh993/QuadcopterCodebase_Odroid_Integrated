// #include <stdio.h>
#include <iostream>
#include <fstream>
#include <signal.h>

#include <vector>

#include "telemetry_lib.h"

// #define LOG_FILE "/home/alarm/data_collection/flight_log.csv"

// #define DEBUG_DATA_LIVE	1

int run_prog;

class Datapoint {
public:
	float timestamp_sec;

	float roll;
	float pitch;
	float yaw;
	float height;

	float roll_cmd;
	float pitch_cmd;
	float height_cmd;

	float motor_1_cmd;
	float motor_2_cmd;
	float motor_3_cmd;
	float motor_4_cmd;

};

static void sig_ctrlC_handler(int dummy)
{
	run_prog = 0;
}

int main(int argc, char** argv)
{
	run_prog = 1;

	signal(SIGINT, sig_ctrlC_handler);

	if(argc < 3)
	{
		// printf("Usage: ./telem_proxy [PORT_PATH]\n");
		std::cout << "Usage: ./telem_proxy [PORT_PATH] [LOGFILE_NAME]\n" << std::endl;
		return -1;
	}

	std::cout << "Attempting to open serial port at: " << argv[1] << "..." << std::endl;

	st_telemetry_channel_init(argv[1]);

	int fd = get_serial_fd();

	if(fd < 0)
	{
		std::cout << "Error opening serial port!!!" << std::endl;
		return -1;
	}

	std::vector<Datapoint> data_buffer;
	float message_contents[TELEMETRY_N_FLOATS_TO_RECV];
	uint8_t raw_telem_buffer[TELEMETRY_N_FLOATS_TO_RECV * 4U];
	uint8_t checksum_received = 0U;

	char start_char = ' ';

	std::cout << "Starting data collection..." << std::endl;

	while(run_prog)
	{
		start_char = recv_byte();
		if(start_char == 's')
		{
			// std::cout << "Got something!!" << std::endl;
			recv_data(raw_telem_buffer, TELEMETRY_N_FLOATS_TO_RECV * 4U);
			checksum_received = recv_byte();
			if(recv_n_floats(message_contents, raw_telem_buffer, checksum_received) == 0)
			{
				Datapoint p;
				p.timestamp_sec = message_contents[0];

				p.roll = message_contents[1];
				p.pitch = message_contents[2];
				p.yaw = message_contents[3];
				p.height = message_contents[4];

				p.roll_cmd = message_contents[5];
				p.pitch_cmd = message_contents[6];
				p.height_cmd = message_contents[7];

				p.motor_1_cmd = message_contents[8];
				p.motor_2_cmd = message_contents[9];
				p.motor_3_cmd = message_contents[10];
				p.motor_4_cmd = message_contents[11];

				data_buffer.push_back(p);

				#ifdef DEBUG_DATA_LIVE
					std::cout << "Roll: " << p.roll << " Pitch: " << p.pitch << " Yaw: " << p.yaw << " Height: " << p.height << std::endl;
				#endif
			}
		}
	}

	std::cout << "Ending data collection" << std::endl;
	std::cout << "Writing to data log file..." << std::endl;

	int num_samples = data_buffer.size();
	int i = 0;

	std::ofstream logfile;
	logfile.open(argv[2]);

	for(i = 0; i < num_samples; ++i)
	{
		logfile << data_buffer[i].timestamp_sec << ", " <<

					data_buffer[i].roll << ", " <<
					data_buffer[i].pitch << ", " <<
					data_buffer[i].yaw << ", " <<
					data_buffer[i].height << ", " <<

					data_buffer[i].roll_cmd << ", " <<
					data_buffer[i].pitch_cmd << ", " <<
					data_buffer[i].height_cmd << ", " <<					

					data_buffer[i].motor_1_cmd << ", " <<
					data_buffer[i].motor_2_cmd << ", " <<
					data_buffer[i].motor_3_cmd << ", " <<
					data_buffer[i].motor_4_cmd << std::endl;
	}

	logfile.close();

	std::cout << "Done writing to log file, bye!!" << std::endl;

	close(fd);

	return 0;
}