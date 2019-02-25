/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/





#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include "cv_bridge/cv_bridge.h"


std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
bool useSampleOutput=false;

using namespace dso;

class PosePublisher : public dso::IOWrap::Output3DWrapper {
public:
  PosePublisher(ros::NodeHandle& nh)
    : pose_pub(nh.advertise<geometry_msgs::PoseStamped>("vodom", 10))
  {}

  void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override {
    Eigen::Quaterniond quat(frame->camToWorld.rotationMatrix());
    Eigen::Vector3d trans = frame->camToWorld.translation();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time(frame->timestamp);
    pose_msg.header.frame_id = "vodom";

    pose_msg.pose.position.x = trans.x();
    pose_msg.pose.position.y = trans.y();
    pose_msg.pose.position.z = trans.z();
    pose_msg.pose.orientation.w = quat.w();
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();

    pose_pub.publish(pose_msg);
  }

private:
  ros::Publisher pose_pub;
};

class PointCloudPublisher : public dso::IOWrap::Output3DWrapper {
public:
  PointCloudPublisher(ros::NodeHandle& nh)
    : points_pub(nh.advertise<sensor_msgs::PointCloud2>("points", 10))
  {}

  // publish point cloud
  void publishKeyframes(std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) {
    if(!final) {
      return;
    }

    double timestamp = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(const auto* frame : frames) {
      timestamp = frame->timestamp;

      double fxi = HCalib->fxli();
      double fyi = HCalib->fyli();
      double cxi = HCalib->cxli();
      double cyi = HCalib->cyli();
      auto const & cam2world=  frame->shell->camToWorld.matrix3x4();

      for (auto const* p : frame->pointHessiansMarginalized) {
      // convert [u, v, depth] to [x, y, z]
        float depth = 1.0f / p->idepth;
        auto const x = (p->u * fxi + cxi) * depth;
        auto const y = (p->v * fyi + cyi) * depth;
        auto const z = depth * (1 + 2*fxi);

        Eigen::Vector3d world_point = cam2world * Eigen::Vector4d(x, y, z, 1.f);

        pcl::PointXYZ pt;
        pt.getVector3fMap() = world_point.cast<float>();
        cloud->push_back(pt);
      }
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;

    if(cloud->empty()) {
        return;
    }

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);
    cloud_msg->header.frame_id = "world";
    cloud_msg->header.stamp = ros::Time(timestamp);

    points_pub.publish(cloud_msg);
  }

private:
  ros::Publisher points_pub;
};

void parseArgument(char* arg)
{
	int option;
	char buf[1000];

	if(1==sscanf(arg,"sampleoutput=%d",&option))
	{
		if(option==1)
		{
			useSampleOutput = true;
			printf("USING SAMPLE OUTPUT WRAPPER!\n");
		}
		return;
	}

	if(1==sscanf(arg,"quiet=%d",&option))
	{
		if(option==1)
		{
			setting_debugout_runquiet = true;
			printf("QUIET MODE, I'll shut up!\n");
		}
		return;
	}


	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}


FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

void vidCb(const sensor_msgs::ImageConstPtr img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
	assert(cv_ptr->image.type() == CV_8U);
	assert(cv_ptr->image.channels() == 1);


	if(setting_fullResetRequested)
	{
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		delete fullSystem;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		fullSystem = new FullSystem();
		fullSystem->linearizeOperation=false;
		fullSystem->outputWrapper = wraps;
	    if(undistorter->photometricUndist != 0)
	    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		setting_fullResetRequested=false;
	}

	MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
	undistImg->timestamp = img->header.stamp.toSec();
	fullSystem->addActiveFrame(undistImg, frameID);
	frameID++;
	delete undistImg;
}

void read_calib(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  ROS_INFO("waiting for camera_info msg...");
  sensor_msgs::CameraInfo::ConstPtr camera_info_msg;
  for(int i=0; i<5; i++) {
  	camera_info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", ros::Duration(10.0));
    if(camera_info_msg == nullptr) {
      ROS_INFO("failed to receive camera_info...");
    } else {
    	break;
    }
  }

  if(camera_info_msg == nullptr) {
    ROS_INFO("failed to receive camera_info...");
    abort();
  }


  bool rectified = pnh.param<bool>("rectified", false);

  calib = "/tmp/dso_calib.txt";
  std::ofstream calib_ofs(calib);
  if(!rectified) {
    calib_ofs << "RadTan "
              << camera_info_msg->K[0] << " " << camera_info_msg->K[4] << " " << camera_info_msg->K[2] << " " << camera_info_msg->K[5] << " "
              << camera_info_msg->D[0] << " " << camera_info_msg->D[1] << " " << camera_info_msg->D[2] << " " << camera_info_msg->D[3] << std::endl;
  } else {
    calib_ofs << "Pinhole "
              << camera_info_msg->K[0] << " " << camera_info_msg->K[4] << " " << camera_info_msg->K[2] << " " << camera_info_msg->K[5] << " 0" << std::endl;
  }
  calib_ofs << camera_info_msg->width << " " << camera_info_msg->height << std::endl;
  calib_ofs << "crop" << std::endl;
  calib_ofs << pnh.param<int>("crop_width", 640) << " " << pnh.param<int>("crop_height", 480) << std::endl;
  calib_ofs.close();
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "dso_live");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

	for(int i=1; i<argc;i++) parseArgument(argv[i]);

  if(calib.empty()) {
    read_calib(nh, pnh);
  }

	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 1200;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=4;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;

	printf("MODE WITH CALIBRATION, but without exposure times!\n");
	setting_photometricCalibration = 2;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;

    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);
    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());


    fullSystem = new FullSystem();
    fullSystem->linearizeOperation=false;


    if(!disableAllDisplay)
	    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1]));


    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


    if(undistorter->photometricUndist != 0)
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    // ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("image", 15, &vidCb);

    fullSystem->outputWrapper.push_back(new PosePublisher(nh));
    fullSystem->outputWrapper.push_back(new PointCloudPublisher(nh));

    ros::spin();

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

    delete undistorter;
    delete fullSystem;

	return 0;
}

