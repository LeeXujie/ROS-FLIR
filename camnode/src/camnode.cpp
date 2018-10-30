#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "FLIR.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camnode");
  ros::NodeHandle nh_("~");
  if(!nh_.ok())return 0;

  int width_, height_;
  std::string camera_frame_id_, camera_name_, camera_info_url_;

  image_transport::ImageTransport it(nh_);
  image_transport::CameraPublisher imgPub = it.advertiseCamera("image_raw", 10);
  boost::shared_ptr<camera_info_manager::CameraInfoManager> caminfo_;

  nh_.param<int>("image_width", width_, 640);
  nh_.param<int>("image_height", height_, 480);
  nh_.param("camera_frame_id", camera_frame_id_, std::string("head_camera"));
  nh_.param("camera_name", camera_name_, std::string("head_camera"));
  nh_.param("camera_info_url", camera_info_url_, std::string(""));

  caminfo_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));

  // check for default camera info
  if (!caminfo_->isCalibrated())
  {
    caminfo_->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = camera_frame_id_;
    camera_info.width = width_;
    camera_info.height = height_;
    caminfo_->setCameraInfo(camera_info);
  }

  FLIR::mGigEGrab cam(width_, height_);
  if(!cam.detectCam())return 0;
  if(!cam.selectCam())return 0;
  if(!cam.openCam())return 0;

  cv::Mat img;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    if(cam.RetrieveBGR(img))
    {
      cv_bridge::CvImage out_msg;
      out_msg.header.stamp=ros::Time::now();
      out_msg.header.frame_id=camera_frame_id_;
      out_msg.encoding=sensor_msgs::image_encodings::BGR8;
      out_msg.image=img;

      sensor_msgs::Image img_;
      out_msg.toImageMsg(img_);

      sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(caminfo_->getCameraInfo()));
      ci->header.frame_id = out_msg.header.frame_id;
      ci->header.stamp = out_msg.header.stamp;

      imgPub.publish(img_, *ci);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
