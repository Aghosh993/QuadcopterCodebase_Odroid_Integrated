#ifndef DETECTOR_H
#define DETECTOR_H

#include "detector_planner_interface.h"
#include "ground_vehicle_estimator.h"


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/video.hpp>

#include <fstream>
#include <signal.h>

#include <iostream>
#include <pthread.h>

#define frame_Width 640
#define frame_Height 480
#define state_variables 4
#define meas_variables 2
#define lines_height_threshold 1.4
#define height_threshold 0.5
#define notFoundCount_threshold 10
#define pad_detection_height_threshold 8
#define idx 0//"/home/naman/Videos/savedvideo20.mkv"
//#define videosavelocation "/home/alarm/data_collection/image_processing_imgs/image-%f.jpg"
#define videosavelocation "/home/naman/Documents/SavedVideos/Video20/image-%f.jpg"

#define landingpad_height 0.5

#define global_coord_enable 1

//#define serial_on 1
//#define visualize 1
//#define datasave 1

//#define csvsave 1

//#define blobdetection_enable 1
//#define framerate

class DetectorInputs;
class DetectorOutputs;

class Detector
{
public:
  Detector();
  
  virtual ~Detector();
  //void update();
  void update(DetectorInputs &detector_inputs, DetectorOutputs &detector_outputs);

private:

        cv::VideoCapture capture;
        cv::Point center;
        
        cv::Point centerdetected;

        cv::Point averagecenter=cv::Point(0,0);
        std::vector<cv::Point> centertracked;
        std::vector<cv::Point> prevcentertracked;

        cv::Mat frame,frame1;
        cv::Mat nextFrame;
        cv::Mat prevFrame;

        cv::Mat cannyfiltered, cannyfiltered1, cannyfiltered2, canny, cannyclone,cannyclone1;
    	std::vector<std::vector<cv::Point>> contours,contours1,contours2;
        std::vector<cv::Vec4i> lines; 
        std::vector<cv::Vec3f> circles;

        bool crashland;
        bool detectionlost;

        int starttracking=0;


        cv::Ptr<cv::ORB> orb = cv::ORB::create();

        std::vector<unsigned char> m_status;
        std::vector<float>         m_error;

        std::vector<cv::KeyPoint> keyImg1,keyImg2;

        int frameCount=0;
        
        int run_prog;

        /* Calibration Matrix*/
        
        //cv::Mat fematrix = (cv::Mat_<float>(3,3) << 5.4220798052687121e+02, 0., 3.1950000000000000e+02, 0., 5.4220798052687121e+02, 2.3950000000000000e+02, 0., 0., 1.);
        //cv:: Mat distcoeff= (cv::Mat_<float>(1,5) << -3.8085850599503479e-01, -2.0280900926602495e-03, 0., 0.,1.5893516544024866e-0);
        

        double gimbal_offset[3] = {0, 0, 0.025};
        double gimbal_angles[2] = {0, 0};

        double UGV_height = 0.50;
        double p_UAV[2] = {0, 0};

        double ang[3] = {0, 0, 0};

        double g_pos[2];
        //cv::Mat fematrix = (cv::Mat_<float>(3,3) <<6.0641852571799518e+02, 0., 4.2350000000000000e+02, 0.,6.0641852571799518e+02, 2.3950000000000000e+02, 0., 0. ,1.);
        //cv::Mat distcoeff = (cv::Mat_<float>(1,5) << 1.0761309127492040e-01,-4.5431041644817438e-01,8.0988110069119754e-03, -5.3957257455037712e-03,4.0780774987473206e-01);
        //cv::Mat fematrixinv=fematrix.inv();

        //cv::Mat rot_camera_uav= (cv::Mat_<float>(3,1) <<0, 0, 0);
        //cv::Mat rot_uav_origin= (cv::Mat_<float>(3,1) <<0, 0, 0);
        //cv::Mat trans_uav_origin= (cv::Mat_<float>(3,1) <<0, 0, 0);
        //cv::Mat trans_camera_uav= (cv::Mat_<float>(3,1) <<0, 0, 0);


        cv::Mat state;  // [x,y,v_x,v_y]
        cv::Mat meas;    // [z_x,z_y]
    	cv::KalmanFilter * kf;

        int averagecount=0;
    	

    	bool firstdetection;
    	bool found;
    	int notFoundCount;

        std::vector<cv::Point> trackingdata;

    	bool setup();
        //bool mainLoop();

        bool mainLoop(QR_State);

        bool KalmanFilterPredictionUpdate();
        bool FindContours();
        bool FindHoughLines();
        bool trackerUpdate();
        bool KalmanFilterMeasurementUpdate();

        inline double square(int);

        double GetMode(double daArray[][3],int,int);
        std::vector<cv::Point2f> featuredetection(int, cv::Ptr<cv::ORB>, cv::Mat, std::vector<cv::KeyPoint>);
        int anglehypoindex(double  anhyin[][3],std::vector<cv::Point2f>,std::vector<cv::Point2f>, std::vector<unsigned char>);
        void checkthreshold(double anhyin[][3],double,double,size_t,cv::Mat,std::vector<cv::Point2f>,float,float);

        #ifdef global_coord_enable
        void getGlobalCoordinates(QR_State);
        cv::Mat getRotationMatrix(QR_State);
        cv::Mat hom_pt;
        #endif
    	
    	#ifdef datasave
        double ticks;
        #endif

        #ifdef csvsave
        std::ofstream pixelfile;
        #endif

        #ifdef framerate
    	double ticks;
    	double precTick;
    	double dT; 
        #endif
};

#endif
