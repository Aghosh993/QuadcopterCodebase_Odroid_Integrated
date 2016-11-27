#ifndef detectionclass_H_
#define detectionclass_H_


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video.hpp>
#include <iostream>
#include <pthread.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

#define frame_Width 640
#define frame_Height 480
#define state_variables 4
#define meas_variables 2
#define lines_height_threshold 1.4
#define height_threshold 0.5
#define notFoundCount_threshold 25
#define idx 0

class detectionclass
{
    public:
    	detectionclass();
    	virtual ~detectionclass();

    	cv::VideoCapture capture;
    	cv::Point center;
        cv::Mat frame;
    	bool crashland;
        bool detectionlost;
    	int start();

        typedef struct {
            uint16_t x;
            uint16_t y;
        } detectedpoint;
        
        union {
        detectedpoint det;
        uint8_t data_output[sizeof(detectedpoint)];
            } detectedpoint_to_data;

        void create_outgoing_packet(uint8_t *);

        //static int serial_port_fd;

    	//void complete();
    	//void shutdown();
        
    protected:
        bool mainLoop();
    private:

    	bool setup();
        //bool mainLoop();

        bool KalmanFilterPredictionUpdate();
        bool FindContours();
        bool FindHoughLines();

        bool KalmanFilterMeasurementUpdate();


        void setup_linux_serial(char* serialPort_name);
        void uart_send_byte_linux(uint8_t xmit_byte);
        uint8_t recv_byte();

        int get_serial_fd(void);



        
    	cv::Mat canny;
    	cv::Mat cannyclone;
    	cv::Mat cannyfiltered;
    	cv::Mat state;  // [x,y,v_x,v_y]
        cv::Mat meas;    // [z_x,z_y]

    	std::vector<std::vector<cv::Point>> contours;
    	std::vector<std::vector<cv::Point>> contours1;
        std::vector<cv::Vec4i> lines; 

    	cv::KalmanFilter * kf;

    	double ticks;
    	double precTick;
    	double dT;

        
        bool firstdetection;
    	bool found;
    	int notFoundCount;      
};

#endif /* detectionclass_H_ */
