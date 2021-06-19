/*********************************************************************************************//**
* @file phoxi_camera_node.cpp
*
* Copyright (c)
* Photoneo s.r.o
* November 2016
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/

#include <phoxi_camera/RosInterface.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "phoxi_camera");
    ros::NodeHandle nh;
    phoxi_camera::RosInterface interface;
    std::string camera_name, camera_info_url;

    if(!nh.getParam("camera_name", camera_name))
      {
	camera_name = "phoxi_camera";
      }
    if(!nh.getParam("camera_info_url", camera_info_url))
      {
	camera_info_url = "/tmp/phoxi_camera.ini";
      }
    
    camera_info_manager::CameraInfoManager cinfo(nh, camera_name, camera_info_url);


    if (cinfo.validateURL(camera_info_url))
      {
	cinfo.loadCameraInfo(camera_info_url);    
      }
    else
      {
	ROS_ERROR("could not validate camera_info_url %s, using default values",camera_info_url.c_str());
	sensor_msgs::CameraInfo ci;
	ci.header.frame_id = "phoxi_frame";
	ci.height = 1544;
	ci.width = 2064;
	ci.distortion_model = "plumb_bob";
	ci.D.push_back(-0.123199); //k1
	ci.D.push_back(0.158915); // k2
	ci.D.push_back(0.000254629); // t1
	ci.D.push_back(8.17743e-05); // t2
	ci.D.push_back(-0.0268664); // t3
	ci.K[0] = 2246.59*0.96; // fx
	ci.K[1] = 0;
	ci.K[2] = 1039.16; // cx
	ci.K[3] = 0;
	ci.K[4] = 2245.66*0.96; // fy
	ci.K[5] = 800.869; // cy
	ci.K[6] = 0;
	ci.K[7] = 0;
	ci.K[8] = 1;
	if(!cinfo.setCameraInfo(ci))
	  {
	    ROS_ERROR("Defaults are faulty. Fix this code");
	  }
      }
    
    std::string ci_topic;
    ci_topic = "/" + camera_name + "/" + "camera_info";
    ros::Publisher cinfo_pub = nh.advertise<sensor_msgs::CameraInfo>(ci_topic.c_str(), 1000);

    int seq=0;
    while(ros::ok())
      {
	interface.gpFrame();
	sensor_msgs::CameraInfo ci = cinfo.getCameraInfo();
	ci.header.stamp = ros::Time::now();
	ci.header.seq = seq++;
	cinfo_pub.publish(ci);
	ros::spinOnce();
      }
    return 0;
}
