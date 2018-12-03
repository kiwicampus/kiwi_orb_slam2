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
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
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

    ImageGrabber igb(&SLAM);


    MultiImagesMemmap video_map("main_stream", MEMMAP_PATH);
    video_map.wait_until_available();

    camera_info_manager::CameraInfoManager cinfo_manager(nh);
    get_rectify_params_calibration(map1, map2, cinfo_manager);

    ros::NodeHandle n;
    ros::Rate r(30.0);

    // message_filters::Subscriber<sensor_msgs::Image> left_sub(nodeHandler, "/camera/left/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> right_sub(nodeHandler, "/camera/right/image_raw", 1);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    // message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    // sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    SLAM.Start();

    while (n.ok())
    {
        
        l_image = video_map.read("L");
        r_image = video_map.read("R");

        if (cinfo_manager.isCalibrated()) {
        remap(l_image, l_image_undist, map1, map2, INTER_LINEAR);
        remap(r_image, r_image_undist, map1, map2, INTER_LINEAR);

        igb.mpSLAM->TrackStereo(l_image_undist,r_image_undist, ros::Time::now().toSec());

        }
        
        ros::spinOnce(); // check for incoming messages
        r.sleep();
    }

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
    do_rectify = false;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        //cv::imshow( "Left", imLeft);
        //cv::imshow( "Right", imRight);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
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

  // Calcualte actual map1 and map2 matrices
  initUndistortRectifyMap(camm, dist, Mat(), camm, Size(cinfo_msg.width,cinfo_msg.height), CV_8UC1, map1, map2);

}