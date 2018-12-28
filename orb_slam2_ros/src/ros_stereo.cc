/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "System.h"
#include "ROSPublisher.h"
#include "utils.h"

#include "easy_memmap.h"

#include <image_geometry/pinhole_camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

// --- Env Variables -------
const float VIDEO_WIDTH = getenv("VIDEO_WIDTH") == NULL ? 320 : std::atof(getenv("VIDEO_WIDTH"));
const float VIDEO_HEIGHT = getenv("VIDEO_HEIGHT") == NULL ? 180 : std::atof(getenv("VIDEO_HEIGHT"));
std::string DISTORTION_PATH = getenv("DISTORSION_PATH") == NULL ? "/usr/src/app/" : getenv("DISTORSION_PATH");
std::string MEMMAP_PATH = getenv("MEMMAP_PATH") == NULL ? "/tmp" : getenv("MEMMAP_PATH");

// ------------------ OpenCV matrices stuff ----------------
Mat l_image, l_image_undist, r_image, r_image_undist;
// Map matrices for undistorting images
Mat map1,map2;

// Functions ---------------
void get_rectify_params_calibration(Mat &map1, Mat &map2, camera_info_manager::CameraInfoManager &cinfo_manager);

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle* nh, float reset_time);

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    cv::Mat M1l2,M2l2,M1r2,M2r2;
    cv::Mat imLeft, imRight;
    cv::Mat imLeft2, imRight2;

    private:
        ros::Subscriber slam_state_sub;
        ros::Timer slam_state_timer;
        float reset_time; // in seconds
        void orb_timer_cb(const ros::TimerEvent &);
        void orb_state_cb(const std_msgs::String::ConstPtr& msg);
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const double freq = 100.0;
    // Set up a namespace for topics
    ros::NodeHandle nh("/orb_slam2");
    ORB_SLAM2::System SLAM(make_unique<ROSSystemBuilder>(argv[1], argv[2], ORB_SLAM2::System::STEREO, freq, nh));

    ImageGrabber igb(&SLAM, &nh, 5.0);



    // MultiImagesMemmap video_map("main_stream", MEMMAP_PATH);
    // video_map.wait_until_available();

    camera_info_manager::CameraInfoManager cinfo_manager(nh);
    // get_rectify_params_calibration(map1, map2, cinfo_manager);

    // get_rectify_params_calibration(igb.M1l, igb.M2l, cinfo_manager);
    // get_rectify_params_calibration(igb.M1r, igb.M2r, cinfo_manager);


    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;
    igb.do_rectify = true;
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);

        cv::initUndistortRectifyMap(K_l,D_l,Mat(),K_l,cv::Size(cols_l,rows_l),CV_32F,igb.M1l2,igb.M2l2);
        cv::initUndistortRectifyMap(K_r,D_r,Mat(),K_r,cv::Size(cols_r,rows_r),CV_32F,igb.M1r2,igb.M2r2);

        ROS_INFO("rectifying image");
    }
    ROS_INFO("not rectifying image");


    ros::NodeHandle nodeHandler;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nodeHandler, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nodeHandler, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    SLAM.Start();


    //cv::namedWindow("L",1);
    //cv::namedWindow("R",1);
    ros::Rate r(30.0);
    while (nodeHandler.ok())
    {   
        if (!igb.imLeft.empty()){
        //cv::imshow( "L", igb.imLeft);
        //cv::imshow( "R", igb.imLeft2);
        waitKey(1);
        }
        
        ros::spinOnce(); // check for incoming messages
        r.sleep();
    }


    // ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    do_rectify = true;
    if(do_rectify)
    {


        // cv::remap(cv_ptrLeft->image,imLeft2,M1l2,M2l2,cv::INTER_LINEAR);
        // cv::remap(cv_ptrRight->image,imRight2,M1r2,M2r2,cv::INTER_LINEAR);
        // cv::imshow( "L", imLeft);
        // cv::imshow( "R", imRight);

        // cv::remap(cv_ptrLeft->image.rowRange(120,360),imLeft,M1l,M2l,cv::INTER_LINEAR);
        // cv::remap(cv_ptrRight->image.rowRange(120,360),imRight,M1r,M2r,cv::INTER_LINEAR);

        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);

        // select a region of interest
        cv::Mat pRoi = imLeft(cv::Rect(0, 0, 640, 120));
        // set roi to some rgb colour   
        pRoi.setTo(cv::Scalar(0, 0, 0));

        // select a region of interest
        cv::Mat pRoi2 = imRight(cv::Rect(0, 0, 640, 120));
        // set roi to some rgb colour   
        pRoi2.setTo(cv::Scalar(0, 0, 0));

        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
        // mpSLAM->TrackStereo(imLeft.rowRange(120,360),imRight.rowRange(120,360),cv_ptrLeft->header.stamp.toSec());

    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}


void get_rectify_params_calibration(cv::Mat &map1, cv::Mat &map2, camera_info_manager::CameraInfoManager &cinfo_manager){

  // Calibration Parameters
  Matx<double, 3, 3> camm;
  Mat dist;
  Matx<double, 3, 4> P;
  Matx33d R;   

  // Basic variables
  std::string camera_parameters_filename, camera_parameters_path;
  float aspect_ratio;

  // Variables for reading camera parameters into CV mats
  image_geometry::PinholeCameraModel model_;

  aspect_ratio = VIDEO_WIDTH/VIDEO_HEIGHT;

  if (aspect_ratio < 1.777) // -> 4/3.0=1.333 en 16/9=1.777
    camera_parameters_filename = "camera_paremeters_640_480.yaml";
  else
    camera_parameters_filename = "camera_paremeters_640_360.yaml";

  camera_parameters_path = "file://" + DISTORTION_PATH + camera_parameters_filename;

  cinfo_manager.loadCameraInfo(camera_parameters_path);

  sensor_msgs::CameraInfo cinfo_msg(cinfo_manager.getCameraInfo());

  // Update the camera model
  model_.fromCameraInfo(cinfo_msg);

  dist = model_.distortionCoeffs();
  camm = model_.intrinsicMatrix();
  P = model_.projectionMatrix();
  R = model_.rotationMatrix();

  // Calcualte actual map1 and map2 matrices
  initUndistortRectifyMap(camm, dist, Mat(), camm, Size(cinfo_msg.width,cinfo_msg.height), CV_8UC1, map1, map2);

   // For stereo
   initUndistortRectifyMap(camm, dist, Mat(), camm, Size(cinfo_msg.width,cinfo_msg.height), CV_8UC1, map1, map2);

}



ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle* nh, float reset_time):
    mpSLAM(pSLAM), reset_time(reset_time) {
    
    slam_state_sub = nh->subscribe("/orb_slam2/state_description", 100, &ImageGrabber::orb_state_cb,this);
    slam_state_timer = nh->createTimer(ros::Duration(10.0), &ImageGrabber::orb_timer_cb, this);

}

void ImageGrabber::orb_timer_cb(const ros::TimerEvent &) {
    ROS_INFO("Resetting ORB slam");
    mpSLAM->Reset();

}

void ImageGrabber::orb_state_cb(const std_msgs::String::ConstPtr& msg){
    if (msg->data.compare("OK") == 0){
        slam_state_timer.setPeriod(ros::Duration(reset_time));
        slam_state_timer.start();
    }
    // else
    //     ROS_INFO("Call back %s", msg->data.c_str());
}










