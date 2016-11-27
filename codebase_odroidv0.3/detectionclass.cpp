#include "detectionclass.hpp"

static int serial_port_fd;

detectionclass::detectionclass()
 	{
    cv::VideoCapture capture;
    cv::Point center;
    //detectedpoint det;
    cv::Mat frame;
 	}

 detectionclass::~detectionclass()
 {
 	cannyfiltered.release();
 
 	capture.release();
 	cv::destroyAllWindows();
 }

 int detectionclass::start()
 {
 	while(!setup());
 		while(mainLoop());
 		return 0;
 }


 bool detectionclass::setup()
 {
    //"/home/naman/Downloads/Video1.mkv"
    if (!capture.open(0))
    {
        std::cout << "Webcam not connected.\n" << "Please verify\n";
        return EXIT_FAILURE;
    }
    
    //set the frame width and height
    capture.set(CV_CAP_PROP_FRAME_WIDTH, frame_Width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, frame_Height);

 	kf = new cv::KalmanFilter(state_variables, meas_variables, 0, CV_32F);
    state.create(state_variables , 1, CV_32F);
    meas.create(meas_variables , 1, CV_32F);
    //cv::Mat procNoise(4, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0 ]
    // [ 0 1 0  dT]
    // [ 0 0 1  0 ]
    // [ 0 0 0  1 ]
    cv::setIdentity(kf->transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf->measurementMatrix = cv::Mat::zeros(meas_variables, state_variables, CV_32F);
    kf->measurementMatrix.at<float>(0) = 1.0f;
    kf->measurementMatrix.at<float>(5) = 1.0f;
    kf->measurementMatrix.at<float>(10) = 1.0f;
    kf->measurementMatrix.at<float>(15) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0]
    // [ 0    Ey  0     0]
    // [ 0    0   Ev_x  0]
    // [ 0    0   0     Ev_y]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf->processNoiseCov.at<float>(0) = 1e-4;
    kf->processNoiseCov.at<float>(5) = 1e-4;
    kf->processNoiseCov.at<float>(10) = 5.0f;
    kf->processNoiseCov.at<float>(15) = 5.0f;
    
    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf->measurementNoiseCov, cv::Scalar(1e-1));

    ticks = 0;
    crashland=false;
    firstdetection=false;
    found = false;  //Flag for when the landing pad is detected
    notFoundCount = 0; //Number of frames for which the landing pad is not detected since the last detection
    detectionlost=true;

    setup_linux_serial("/dev/ttyUSB0");
    return true;
 }

 bool detectionclass::mainLoop()
{   
    char ch=0;
    int height=2;
    
    precTick = ticks;
    ticks = (double) cv::getTickCount();
    dT=(ticks - precTick) / cv::getTickFrequency();
    //std::cout<<1/dT<<std::endl;

    capture>>frame;
    
    if(!firstdetection || height>lines_height_threshold)
    {

        
        while(!KalmanFilterPredictionUpdate());
        while(!FindContours());
        while(!KalmanFilterMeasurementUpdate());

    }

    else if(!detectionlost && height<lines_height_threshold )
    {

        while(!KalmanFilterPredictionUpdate());
        while(!FindHoughLines());
        while(!KalmanFilterMeasurementUpdate());
    }
    
    else if(!detectionlost && height<height_threshold)
    {
        crashland=true;

    }
    else if(firstdetection && notFoundCount>notFoundCount_threshold && height<lines_height_threshold)
    {
        crashland=false;
    }    
	
    //cv::imshow("Frame", frame);
    ch=cv::waitKey(1);
    
    

    return true;
    /*    char ch=0;
    while (ch != 'q' && ch != 'Q')
    {
        precTick = ticks;
        ticks = (double) cv::getTickCount();
        dT=(ticks - precTick) / cv::getTickFrequency();

        capture>>frame;

        while(!KalmanFilterPredictionUpdate());
        while(!FindContours());
        while(!KalmanFilterMeasurementUpdate());
        
        //std::cout<<found<<std::endl;
        loopcomplete();
        ch=cv::waitKey(1);
    }
	return true;*/
}


bool detectionclass::KalmanFilterPredictionUpdate()
{
	if (!detectionlost)
        {
            // >>>> Matrix A
            //Update the transition matrix
            kf->transitionMatrix.at<float>(2) = dT;
            kf->transitionMatrix.at<float>(7) = dT;
            // <<<< Matrix A

            state = kf->predict();
            
            detectedpoint_to_data.det.x = state.at<float>(0);
            detectedpoint_to_data.det.y = state.at<float>(1);
            
            if (get_serial_fd() != -1) {

                uint8_t packet[sizeof(detectedpoint)+1];

                create_outgoing_packet(packet);

                for(int i=0; i<sizeof(detectedpoint)+1; i++) {
                // printf("Sending byte %d > %c\n", i, packet[i]);
                uart_send_byte_linux(packet[i]);
                // process_incoming_gimbal_data(recv_byte());
                    }
                }


            //std::cout<<detectedpoint_to_data.data_output<<std::endl;
            //cv::circle(frame, center,6, CV_RGB(0,0,255), 2, 8, 0 );
            
            //cv::Mat hom_pt= (cv::Mat_<float>(3,1) <<state.at<float>(0), state.at<float>(1), 1);
            //hom_pt = fematrixinv*hom_pt; 
            //hom_pt *= 1/cv::norm(hom_pt);
            //cv::rectangle(img, predRect, CV_RGB(255,0,0), 2);
        }
        return true;
}


bool detectionclass::FindContours()
{
    found=false;
	cv::cvtColor(frame, canny, CV_BGR2GRAY);
    cv::Canny(canny, canny, 350, 50, 3);
        //Clone the canny edge because find contour messes the frame up
        cannyclone=canny.clone();
        
        // Find all the contours in the canny edge frame
        cv::findContours(canny, contours, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

        std::vector<std::vector<cv::Point>> contours_poly( contours.size() );
        cv::Mat drawing = cv::Mat::zeros( cannyclone.size(), CV_8UC1 );

        for( int i = 0; i < contours.size(); i++ )
        {
            //Approximate a polynomial for each contour
            cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 1, true);

            // Check if the contour is convex and has greater than 12
            if(contours_poly[i].size()>12 && isContourConvex(contours_poly[i]))
                cv::drawContours( drawing, contours_poly, i, CV_RGB(255,255,255), -1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        }

        cannyclone.copyTo(cannyfiltered, drawing);
        
        cv::dilate(cannyfiltered,cannyfiltered,cv::Mat(), cv::Point(-1, -1), 1);

        cv::findContours(cannyfiltered, contours1, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
            
        std::vector<cv::Moments> mu(contours1.size() );
        std::vector<cv::Point> mc( contours1.size() );
            int j=0;
            double area;
            for( int i = 0; i < contours1.size(); i++ )
            { 
                cv::approxPolyDP( cv::Mat(contours1[i]), contours1[i], 2, true);
                if(contours1[i].size()>8 && isContourConvex(contours1[i]))
                {
                    if(contourArea(contours1[i],false)>area)
                    {
                    mu[j] = moments( contours1[i], true ); 
                    mc[j] = cv::Point( mu[j].m10/mu[j].m00 , mu[j].m01/mu[j].m00 ); 
                        if((int)mc[j].x!=0)
                        {
                        center=mc[j];
                        found=true;
                        }
                    j=j+1;
                    }
                }
            }
            return true;
}

bool detectionclass::FindHoughLines()
{
    found=false;
    cv::cvtColor(frame, canny, CV_BGR2GRAY);
    cv::Canny(canny, canny, 350, 50, 3);
    HoughLinesP(canny, lines, 1, CV_PI/180, 40, 3,50);
    if(lines.size()>0)
    {
        found=true;
        center=cv::Point(0,0);
        for(int i=0; i<lines.size();i++)
        {
            center+= cv::Point((lines[i][0]+lines[i][2])/2,(lines[i][1]+lines[i][3])/2);
        }
        center=center/(int)lines.size();
    }
    return true;
}

bool detectionclass::KalmanFilterMeasurementUpdate()
{
	if(!found)
    {
        {
             notFoundCount++; 
             if( notFoundCount > notFoundCount_threshold)
            {
                detectionlost=true;
                notFoundCount=notFoundCount_threshold+1;
            }
        }
    }

    else
          {
            notFoundCount = 0;
            detectionlost=false;
            meas.at<float>(0) = (float)center.x;
            meas.at<float>(1) = (float)center.y;

            if (!firstdetection) // First detection!
            {
                // >>>> Initialization
                kf->errorCovPre.at<float>(0) = 1; // px
                kf->errorCovPre.at<float>(5) = 1; // px
                kf->errorCovPre.at<float>(10) = 1;
                kf->errorCovPre.at<float>(15) = 1;

                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                // <<<< Initialization

                firstdetection = true;
            }
            else
                kf->correct(meas); // Kalman Correction
        }
        cannyfiltered.release();
    return true;
}

void detectionclass::setup_linux_serial(char* serialPort_name)
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


void detectionclass::uart_send_byte_linux(uint8_t xmit_byte)
{
    write(serial_port_fd, &xmit_byte, 1);
}

uint8_t detectionclass::recv_byte()
{
    int bytes_read = 0;
    char read_char;
    while(bytes_read < 1)
    {
        bytes_read = read(serial_port_fd, &read_char, 1);
    }
    return bytes_read;
}

int detectionclass::get_serial_fd(void)
{
    return serial_port_fd;
}

void detectionclass::create_outgoing_packet(uint8_t *outgoing_data_buffer)
{
    outgoing_data_buffer[0] = 's';

    for(int i=0; i<sizeof(detectedpoint); ++i) {
                outgoing_data_buffer[i+1U] = detectedpoint_to_data.data_output[i];
        }
}