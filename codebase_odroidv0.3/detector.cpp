#include "detector.h"

static const double pi = 3.14159265358979323846;

Detector::Detector()
    {
    while(!setup());
    }

 Detector::~Detector()
 {
    cannyfiltered.release();
    
    capture.release();
    cv::destroyAllWindows();
 }

/* Function for setting up all the parameters

    1) Release previously captured camera port (Used to remove memory leak caused by pressing control+C
    2) Start the camera and set its height and width
    3) Initialize the kalman filter parameters for tracking
    4) Initialize all the flags
    5) Initialize serial communincation if serial_on is defined*/
 bool Detector::setup()
 {
    //"/home/naman/Downloads/Video1.mkv"
    capture.release();

     cv::waitKey(10);
    //std::cout<<"A"<<std::endl;
    // /home/naman/Documents/Videos/Video3 7 8 9
     // 1 2 4 5 6
    if (!capture.open(idx))
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
    //kf->measurementMatrix.at<float>(10) = 0.0f;
    //kf->measurementMatrix.at<float>(15) = 0.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0]
    // [ 0    Ey  0     0]
    // [ 0    0   Ev_x  0]
    // [ 0    0   0     Ev_y]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf->processNoiseCov.at<float>(0) = 50;
    kf->processNoiseCov.at<float>(5) = 50;
    kf->processNoiseCov.at<float>(10) = 15;
    kf->processNoiseCov.at<float>(15) = 15;
    
    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf->measurementNoiseCov, cv::Scalar(15));

    //ticks = 0;
    crashland=false; //Flag for to tell the UAV whether to crashland
    firstdetection=false; // Flag is set to true when the landing pad is detected for the first time using Landing Pad Detection Algorithm
    found = false;  //Flag for when the landing pad is detected
    notFoundCount = 0; //Number of frames for which the landing pad is not detected since the last detection
    detectionlost=true; // Flag is set to true if landing pad is not detected initially or when the UAV is not able to find the landning pad for 40 frames
    #ifdef framerate
     ticks=0;
    #endif
    return true;
 }

 bool Detector::mainLoop(QR_State stateupdate)
{   
    char ch=0;
    float height= 3; //Height of the UAV to be updated when data comes through the Autopilot
    #ifdef datasave
    ticks = (double) cv::getTickCount();
    #endif

    #ifdef framerate
     
    precTick = ticks;
    ticks = (double) cv::getTickCount();
     dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
    std::cout << 1/dT << std::endl;
    #endif
    

    capture>>frame;  // Copies the frame of the camera to matrix frame

    if( frame.empty() )
            return false;
        
    /* This runs if the first detection has not yet taken place or the height is greater than the height where circle can be detected*/
    if(!firstdetection || height>lines_height_threshold)
    {

        
    while(!KalmanFilterPredictionUpdate()); // Kalman Filter Prediction Update
    while(!FindContours());  // Detection of landing pad circles using contours
    while(!KalmanFilterMeasurementUpdate()); // Kalman Filter Measurement update
   
    }
    //else if(!detectionlost && height<lines_height_threshold )
    /* This runs if the first detection has already taken place and the height is less than the lines height threshold*/
    else if(firstdetection && height<lines_height_threshold )
    {

        while(!KalmanFilterPredictionUpdate()); // Kalman Filter Prediction Update
        while(!FindHoughLines()); // Detects lines in the landing pad and sends their intersection to the measurement update
        while(!KalmanFilterMeasurementUpdate()); // Kalman Filter Measurement update

    }
    #ifdef visualize
    //cv::imshow("Frame", cannyclone);
    cv::imshow("Frame1", frame);
    #endif

    #ifdef datasave
    char buffer[200];
    sprintf(buffer,videosavelocation,ticks);
    cv::imwrite(buffer, frame);
    #endif  

    ch=cv::waitKey(1);
    

    return true;
    
}

bool Detector::FindContours()
{

    found=false;

    cv::cvtColor(frame, canny, CV_BGR2GRAY); //Convert the Colour Image to grayscale (Works better on the Canny Edge)
    //int linescount=0;
    //cv::Point tempcenter1=cv::Point(0,0);
    cv::HoughCircles( canny, circles, CV_HOUGH_GRADIENT, 1, 500, 150, 40,40,400);

    cv::Mat drawing = cv::Mat::zeros( canny.size(), CV_8UC1 );
    if(circles.size()>0)
    {
    center=cv::Point(circles[0][0], circles[0][1]);
    found=true;
    #ifdef visualize
    cv::circle(frame, center,8, CV_RGB(255,0,0), 2, 8, 0 );
    #endif
    cv::circle( drawing, cv::Point(circles[0][0], circles[0][1]),  circles[0][2]-25, cv::Scalar(255,255,255), -1, 8, 0 );
    }
    else
        center=cv::Point(0,0);

                    //cv::circle( frame,center,  6, cv::Scalar(0,0,255), 2, 8, 0 );
                    
/*
            if(found==false)
            {
                cv::Mat canny;
    std::vector<cv::Vec4i> lines; 

    //cv::cvtColor(frame, canny, CV_BGR2GRAY);
    cv::Canny(frame, canny, 350, 75, 3);

    // Detect the lines in the frame
    HoughLinesP(canny, lines, 1, CV_PI/180, 40, 30,50);

    cv::Point tempcenter=cv::Point(0,0);
    std::vector<bool> linesdone(lines.size(),false);

                if(lines.size()>1)
                {
                 int linescount=0;   
                    for(int i=0; i<lines.size();i++)
                    {
                        //cv::line(frame,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),CV_RGB(255,255,0), 2, 8, 0);
                        //cv::Vec3b color1=frame.at<cv::Vec3b>(cv::Point(lines[i][0],lines[i][1])),color2=frame.at<cv::Vec3b>(cv::Point(lines[i][2],lines[i][3]));
                        //if(color1[0]<10 && color1[1]<10 && color1[2]<10 && color2[0]<10 && color2[1]<10 && color2[2]<10)
                        //{
                        linesdone[i]=true;
                        if(lines[i][2]-lines[i][0]==0)
                            continue;
                        int m=(lines[i][3]-lines[i][1])/(lines[i][2]-lines[i][0]);
                        //int b=lines[i][1]-(m*lines[i][0]);
                        for(int j=i+1; j<lines.size();j++) 
                        {  
                            if(linesdone[j]==false)
                            {
                                //cv::Vec3b color1=frame.at<cv::Vec3b>(cv::Point(lines[j][0],lines[j][1])),color2=frame.at<cv::Vec3b>(cv::Point(lines[j][2],lines[j][3]));
                            if((lines[j][3]-lines[j][1]==0)) //|| color1[0]>10 && color1[1]>10 && color1[2]>10 && color2[0]>10 && color2[1]>10 && color2[2]>10)
                            {
                                linesdone[j]=true;
                                continue;
                            }
                            int m1= (lines[j][2]-lines[j][0])/(lines[j][3]-lines[j][1]);
                            if(abs(m+m1)<1)
                            {
                                //int a=lines[j][1]+(m*lines[j][0]);
                                linesdone[j]==true;
                                linescount++;
                                cv::line(frame,cv::Point(lines[j][0],lines[j][1]),cv::Point(lines[j][2],lines[j][3]),CV_RGB(255,0,255), 2, 8, 0);
                                //tempcenter.x+=((a-b)*m)/((m*m)+1);
                                //tempcenter.y+=(b+((m*m)*a))/(m*m+1);
                                tempcenter.x+=(lines[j][0]+lines[j][2])/2;
                                tempcenter.y+=(lines[j][1]+lines[j][3])/2;
                                found=true;
                            }
                            }

                        }
                        //}
                    }
                    if(found==true)
                    {
                        center=tempcenter/linescount;
                    }
                }
            }*/

            //cannyfiltered.release();
            //cannyfiltered1.release();
            //cannyfiltered2.release();

            return true;
}


bool Detector::KalmanFilterPredictionUpdate()
{

    if (!detectionlost)
        {
            // >>>> Matrix A
            //Update the transition matrix with the time Elapsed
            kf->transitionMatrix.at<float>(2) = 0.033;
            kf->transitionMatrix.at<float>(7) = 0.033;
            // <<<< Matrix A

            // Do a prediction update
            state = kf->predict();

             //Condition for starting tracking using Kalman filter and giving it time to initialize with a good estimate
                //Send the detected point initially and not the prediction update untill the below condition is not fullfilled

            if(starttracking<15)
            //if(starttracking<4 && center.x+center.y-state.at<float>(0)-state.at<float>(1)<200)
            {
            if(found)
            {
            starttracking=starttracking+1;

            centerdetected.x=center.x;
            centerdetected.y=center.y;

            //cv::circle(frame, centerdetected,6, CV_RGB(0,255,0), 2, 8, 0 );
            }
             //If the point is not detected for= 3 consecutive frames change the counter again to zero
            /*if(notFoundCount>2)
            {
                starttracking=0;
                
            }*/
            }




            // Start using prediction updates for tracking
            else if(starttracking>=15)
            {

            if(trackingdata.size()>2)
            {
                averagecenter=(trackingdata[trackingdata.size()-1]+trackingdata[trackingdata.size()-2]+trackingdata[trackingdata.size()-3])/3;

            if((averagecenter.x-state.at<float>(0))*(averagecenter.x-state.at<float>(0))+(averagecenter.y-state.at<float>(1))*(averagecenter.y-state.at<float>(1))<6400)
            {
            centerdetected.x=state.at<float>(0);
            centerdetected.y=state.at<float>(1);
            }
            else
            {
            centerdetected.x=centerdetected.x+(state.at<float>(0)-centerdetected.x)/2;
            centerdetected.y=centerdetected.y+(state.at<float>(1)-centerdetected.y)/2;
            }
            }

            




            
            //#ifdef visualize
            //std::cout<<center<<std::endl;
           //cv::circle(frame, centerdetected,6, CV_RGB(0,255,0), 2, 8, 0 );
           //#endif
            }
            
            //cv::Mat hom_pt= (cv::Mat_<float>(3,1) <<state.at<float>(0), state.at<float>(1), 1);
            //hom_pt = fematrixinv*hom_pt; 
            //hom_pt *= 1/cv::norm(hom_pt);
            //cv::rectangle(img, predRect, CV_RGB(255,0,0), 2);
            trackingdata.push_back(centerdetected);
        }
        return true;
}


bool Detector::FindHoughLines()
{
    found=false;
    // Do the same canny edge detection as in the findcontours function

    cv::Mat canny;
    std::vector<cv::Vec4i> lines; 

    //cv::cvtColor(frame, canny, CV_BGR2GRAY);
    cv::Canny(frame, canny, 150, 40, 3);

    // Detect the lines in the frame
    HoughLinesP(canny, lines, 1, CV_PI/180, 120, 3,50);

    cv::Point tempcenter=cv::Point(0,0);
    std::vector<bool> linesdone(lines.size(),false);

                if(lines.size()>1)
                {
                 int linescount=0;   
                    for(int i=0; i<lines.size();i++)
                    {
                        //cv::line(frame,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),CV_RGB(255,255,0), 2, 8, 0);
                        //cv::Vec3b color1=frame.at<cv::Vec3b>(cv::Point(lines[i][0],lines[i][1])),color2=frame.at<cv::Vec3b>(cv::Point(lines[i][2],lines[i][3]));
                        //if(color1[0]<10 && color1[1]<10 && color1[2]<10 && color2[0]<10 && color2[1]<10 && color2[2]<10)
                        //{
                        linesdone[i]=true;
                        if(lines[i][2]-lines[i][0]==0)
                            continue;
                        float m=(lines[i][3]-lines[i][1])/(lines[i][2]-lines[i][0]);
                        float b=lines[i][1]-(m*lines[i][0]);
                        for(int j=i+1; j<lines.size();j++) 
                        {  
                            if(linesdone[j]==false)
                            {
                                //cv::Vec3b color1=frame.at<cv::Vec3b>(cv::Point(lines[j][0],lines[j][1])),color2=frame.at<cv::Vec3b>(cv::Point(lines[j][2],lines[j][3]));
                            if((lines[j][3]-lines[j][1]==0) || (lines[j][2]-lines[j][0])==0 ) //|| color1[0]>10 && color1[1]>10 && color1[2]>10 && color2[0]>10 && color2[1]>10 && color2[2]>10)
                            {
                                linesdone[j]=true;
                                continue;
                            }
                            float m1= (lines[j][3]-lines[j][1])/(lines[j][2]-lines[j][0]);
                            if(abs(m-(1/m1))<1)
                            {
                                float a=lines[j][1]-(lines[j][0]*m1);
                                linesdone[j]==true;
                                linescount++;
                                //cv::line(frame,cv::Point(lines[j][0],lines[j][1]),cv::Point(lines[j][2],lines[j][3]),CV_RGB(255,0,255), 2, 8, 0);
                                //tempcenter.x+=((a-b)*m)/((m*m)+1);
                                //tempcenter.y+=(b+((m*m)*a))/(m*m+1);
                                tempcenter.x+=(lines[j][0]+lines[j][2])/2;
                                tempcenter.y+=(lines[j][1]+lines[j][3])/2;
                                found=true;
                            }
                            }

                        }
                        //}
                    }
                    if(found==true )
                    {
                        center=tempcenter/linescount;
                    }
                }
    return true;
}

bool Detector::KalmanFilterMeasurementUpdate()
{
    // Increase the not found count
    // Increase the not found coun
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
                kf->errorCovPre.at<float>(0) = 200; // px
                kf->errorCovPre.at<float>(5) = 200; // px
                kf->errorCovPre.at<float>(10) = 100;
                kf->errorCovPre.at<float>(15) = 100;

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
        
        #ifdef visualize
            //std::cout<<center<<std::endl;
         if((int)centerdetected.x!=0 && !detectionlost)
            {

            cv::circle(frame, centerdetected,8, CV_RGB(0,255,255), 2, 8, 0 );
            //getGlobalCoordinates();
            }
        #endif
    return true;
}

inline double Detector::square(int a)
{
    return a * a;
}

double Detector::GetMode(double daArray[][3],int iSize,int k)
{
    // Allocate an int array of the same size to hold the
    // repetition count
    int* ipRepetition = new int[iSize];
    #pragma omp parallel for
    for (int i = 0; i < iSize; ++i) {
        ipRepetition[i] = 0;
        int j = 0;
        bool bFound = false;
        while ((j < i) && (daArray[i][k] != daArray[j][k])) {
            if (daArray[i][k] != daArray[j][k]) {
                ++j;
            }
        }
        ++(ipRepetition[j]);
    }
    int iMaxRepeat = 0;
    
    for (int i = 1; i < iSize; ++i) {
        if (ipRepetition[i] > ipRepetition[iMaxRepeat]) {
            iMaxRepeat = i;
        }
    }
    delete [] ipRepetition;
    return daArray[iMaxRepeat][k];
}

std::vector<cv::Point2f> Detector::featuredetection(int count, cv::Ptr<cv::ORB> orb1, cv::Mat img, std::vector<cv::KeyPoint> keypt)
{
    std::vector<cv::Point2f> corners;
    if(count==0)
        {
        orb1->detect(img,keypt);

        for(size_t i=0;i<keypt.size();i++)
        {
            corners.push_back(keypt[i].pt);
        }
        }
        return corners;
}

int Detector::anglehypoindex(double anhyin[][3],std::vector<cv::Point2f> prev_corner,std::vector<cv::Point2f>cur_corner, std::vector<unsigned char> status)
{       
    int j=0;
        for( size_t i = 0; i < prev_corner.size(); i++ )
        { 
            anhyin[j][1] = sqrt( square(prev_corner[i].y - cur_corner[i].y) + square(prev_corner[i].x - cur_corner[i].x) );
            if(anhyin[j][1]<20 && status[i])
            {
            anhyin[j][0] = atan2( (double) prev_corner[i].y - cur_corner[i].y, (double) prev_corner[i].x - cur_corner[i].x );
            anhyin[j][2]=i;
            j=j+1;
            }
            
        }
        return j;
}

void Detector::checkthreshold(double anhyin[][3],double thres,double thres1,size_t j,cv::Mat img,std::vector<cv::Point2f> cur_corner,float lim,float lim1)
{
    //cv::Point2f center=cv::Point2f(0,0);
    //int counter=0;

    std::vector<cv::Point> points;
    //#pragma omp parallel for
    for( size_t i = 0; i < j; i++ )
        {       
                
                if(abs(anhyin[i][1]-thres1)>lim1 && abs(anhyin[i][0]-thres)>lim && anhyin[i][1]<6)
                {

                    //center+=cur_corner[anhyin[i][2]];
                    points.push_back(cur_corner[anhyin[i][2]]);
                    //cv::circle(img, cur_corner[anhyin[i][2]],6, CV_RGB(255,0,0), 2, 8, 0 );
                    //counter++;
                 }
                
        }
        
        std::vector<bool> pointsfound(points.size(), false);

        std::vector<cv::Point> centerPoints;
        for( size_t i = 0; i < points.size(); i++ )
        {
            //cv::circle(img, points[i],6, CV_RGB(255,0,0), 2, 8, 0 );
            cv::Point temppoint=points[i];
            int k=1;
            if(pointsfound[i]==true)
                continue;
            pointsfound[i]=true;
            for (size_t j = i+1; j < points.size(); j++)
            {
                if(pointsfound[j]!=true && (abs(points[j].x-points[i].x)<40 && abs(points[j].y-points[i].y)<40))
                {
                    pointsfound[j]=true;
                    k=k+1;
                    temppoint+=points[j];
                }
            }
            if(i==0)
            {
            centerPoints.push_back((temppoint)/k);
            }
            else
            {
                size_t l;
            for( l = 0; l < centerPoints.size(); l++ )
            {
                temppoint=temppoint/k;
                if((abs(centerPoints[l].x-temppoint.x)<40 && abs(centerPoints[l].y-temppoint.y)<40))
                {
                    centerPoints[l]+=temppoint;
                    break;
                }

            }
            if(l==centerPoints.size()) 
                 centerPoints.push_back(temppoint);
            }         
            //cv::circle(img, temppoint,6, CV_RGB(i*10,0,255), 2, 8, 0 );
        }

        for( size_t i = 0; i < centerPoints.size(); i++ )
        {
            cv::circle(img, centerPoints[i],6, CV_RGB(i*10,0,255), 2, 8, 0 );
        }
        /*
        if(counter!=0)
        {
            center/=counter;
            //cv::circle(img, center,6, CV_RGB(0,0,255), 2, 8, 0 );
        }*/
}


void Detector::update(DetectorInputs &detector_inputs, DetectorOutputs &detector_outputs)
{
    static QR_State stateupdate;
    stateupdate=*(detector_inputs.qr_state);

    while(!mainLoop(stateupdate));

    #ifdef global_coord_enable
    getGlobalCoordinates(stateupdate);
    #endif

   detector_outputs.px=(centerdetected.x-((float)frame_Width/2))/((float)frame_Width/2);
   detector_outputs.py=(centerdetected.y-((float)frame_Height/2))/((float)frame_Height/2);

   //detector_outputs.px=(centerdetected.x);
   //detector_outputs.py=(centerdetected.y);

   //detector_outputs.vx_b= state.at<float>(2);
    //detector_outputs.vy_b=state.at<float>(3);

   #ifdef global_coord_enable
   detector_outputs.x_b=g_pos[0];
   detector_outputs.y_b=g_pos[1];

   detector_outputs.vx_b=0;
   detector_outputs.vy_b=0;
   detector_outputs.first_detection=firstdetection;
   detector_outputs.not_found=detectionlost;
   #endif

}
/*
void Detector::update()
{
    while(!mainLoop());
}*/

#ifdef global_coord_enable
void Detector::getGlobalCoordinates(QR_State stateupdate)
{
    CameraMatrix cm;
        cm.cx = 319.5;
        cm.cy = 239.5;
        cm.fx = 487.2;
        cm.fy = 486.4;
    ang[0]=stateupdate.ang_x;
    ang[1]= stateupdate.ang_y;
    ang[2]= stateupdate.ang_z;
    double heigh=stateupdate.z-UGV_height;
    //hom_pt = getRotationMatrix(stateupdate)*fematrixinv*(cv::Mat_<float>(3,1) <<centerdetected.x, centerdetected.y, 1);
    //cv::Mat tempMat2= rot_camera_uav*trans_camera_uav;
    estimate_position(centerdetected.x, centerdetected.y, cm, gimbal_angles, gimbal_offset,ang, p_UAV, heigh, g_pos);
    //hom_pt *= 1/cv::norm(hom_pt);
    //hom_pt*=(-(stateupdate.z-0.50)/hom_pt.at<float>(2,0));
    
    //hom_pt=rot_camera_uav*hom_pt;

    //std::cout<<hom_pt.at<float>(1,0)<<std::endl;

    //std::usleep(200);

}

cv::Mat Detector::getRotationMatrix(QR_State stateupdate)
{   
    //UAV Quaternion to rotation matrix
    /*cv::Mat rot_mat=cv::Mat<float>(3,3)<<;*/

    //UAV Roation  matrix multiplied with Gimbal Rotation matrix multiplied with Heading Bias
}
#endif