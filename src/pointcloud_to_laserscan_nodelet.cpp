/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace pointcloud_to_laserscan
{

  PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet() {}

  void PointCloudToLaserScanNodelet::onInit()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<std::string>("target_frame", target_frame_, "");
    private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
    private_nh_.param<double>("min_height", min_height_, -0.8);  // for legs
    private_nh_.param<double>("max_height", max_height_, -0.6);  // for legs

    private_nh_.param<double>("angle_min", angle_min_, -M_PI);
    private_nh_.param<double>("angle_max", angle_max_, M_PI);
    private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 360.0);
    private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
    private_nh_.param<double>("range_min", range_min_, 0.45);
    private_nh_.param<double>("range_max", range_max_, 20.0);

    private_nh_.param<double>("min_height", torso_min_height_, 0.0);  // for torso
    private_nh_.param<double>("max_height", torso_max_height_, 0.2);  // for torso

    int concurrency_level;
    private_nh_.param<int>("concurrency_level", concurrency_level, 1);
    private_nh_.param<bool>("use_inf", use_inf_, true);

    //Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
    if (concurrency_level == 1)
    {
      nh_ = getNodeHandle();
    }
    else
    {
      nh_ = getMTNodeHandle();
    }

    // Only queue one pointcloud per running thread
    if (concurrency_level > 0)
    {
      input_queue_size_ = concurrency_level;
    }
    else
    {
      input_queue_size_ = boost::thread::hardware_concurrency();
    }

    // if pointcloud target frame specified, we need to filter by transform availability
    if (!target_frame_.empty())
    {
      tf2_.reset(new tf2_ros::Buffer());
      tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
      message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
      message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
      message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
    }
    else // otherwise setup direct subscription
    {
      sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    }

    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10//,
            //boost::bind(&PointCloudToLaserScanNodelet::connectCb, this)//,
            //boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this)
            );

    torso_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_ubg", 10//,
            //boost::bind(&PointCloudToLaserScanNodelet::connectCb, this)//,
            //boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this)
            );

    sub_.subscribe(nh_, "/velodyne_points", input_queue_size_); // added by aGn.
  }

  void PointCloudToLaserScanNodelet::connectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    //if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
    //{
      NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
      sub_.subscribe(nh_, "/velodyne_points", input_queue_size_);
    //}
  }

  void PointCloudToLaserScanNodelet::disconnectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    //if (pub_.getNumSubscribers() == 0)
    //{
      NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
      sub_.unsubscribe();
    //}
  }

  void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                               tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  {
    NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
        << message_filter_->getTargetFramesString());
  }

  void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {

    //build laserscan output
    sensor_msgs::LaserScan legs_output;
    sensor_msgs::LaserScan torso_output;
    legs_output.header = cloud_msg->header;
    torso_output.header = cloud_msg->header;
    if (!target_frame_.empty())
    {
      legs_output.header.frame_id  = target_frame_;  // TODO :: must be change.
      torso_output.header.frame_id = target_frame_;  // TODO :: must be change.
    }

    legs_output.angle_min = angle_min_;
    legs_output.angle_max = angle_max_;
    legs_output.angle_increment = angle_increment_;
    legs_output.time_increment = 0.0;
    legs_output.scan_time = scan_time_;
    legs_output.range_min = range_min_;
    legs_output.range_max = range_max_;

    torso_output.angle_min = angle_min_;
    torso_output.angle_max = angle_max_;
    torso_output.angle_increment = angle_increment_;
    torso_output.time_increment = 0.0;
    torso_output.scan_time = scan_time_;
    torso_output.range_min = range_min_;
    torso_output.range_max = range_max_;

    //determine amount of rays to create
    uint32_t ranges_size = std::ceil((legs_output.angle_max - legs_output.angle_min) / legs_output.angle_increment);
//    uint32_t ranges_size = std::ceil(1440);  // aGn: Like a Hokuyo-UTM

    //determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_)
    {
        legs_output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
        torso_output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    }
    else
    {
        legs_output.ranges.assign(ranges_size, legs_output.range_max + 1.0);
        torso_output.ranges.assign(ranges_size, torso_output.range_max + 1.0);
    }

    sensor_msgs::PointCloud2ConstPtr cloud_out;
    sensor_msgs::PointCloud2Ptr cloud;

    // Transform cloud if necessary
    if (!(legs_output.header.frame_id == cloud_msg->header.frame_id))  // TODO
    {
      try
      {
        cloud.reset(new sensor_msgs::PointCloud2);
        tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
        cloud_out = cloud;
      }
      catch (tf2::TransformException ex)
      {
        NODELET_ERROR_STREAM("Transform failure: " << ex.what());
        return;
      }
    }
    else
    {
      cloud_out = cloud_msg;
    }

    // Iterate through pointcloud to create legs scan
    for (sensor_msgs::PointCloud2ConstIterator<float>
              iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
              iter_x != iter_x.end();
              ++iter_x, ++iter_y, ++iter_z)
    {

      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
      {
        NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
        continue;
      }

      if (*iter_z > max_height_ || *iter_z < min_height_)
      {
        NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
        continue;
      }

      double range = hypot(*iter_x, *iter_y);
      if (range < range_min_)
      {
        NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
                      *iter_z);
        continue;
      }

      double angle = atan2(*iter_y, *iter_x);
      if (angle < legs_output.angle_min || angle > legs_output.angle_max)
      {
        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, legs_output.angle_min,
                legs_output.angle_max);
        continue;
      }

      //overwrite range at laserscan ray if new range is smaller
      int index = (angle - legs_output.angle_min) / legs_output.angle_increment;
      if (range < legs_output.ranges[index])
      {
        legs_output.ranges[index] = range;
      }

    }

    // Iterate through pointcloud to create the torso scan.
    for (sensor_msgs::PointCloud2ConstIterator<float>
                   iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
           iter_x != iter_x.end();
           ++iter_x, ++iter_y, ++iter_z)
      {

          if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
          {
              NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
              continue;
          }

          if (*iter_z > torso_max_height_ || *iter_z < torso_min_height_)
          {
              NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
              continue;
          }

          double range = hypot(*iter_x, *iter_y);
          if (range < range_min_)
          {
              NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
                            *iter_z);
              continue;
          }

          double angle = atan2(*iter_y, *iter_x);
          if (angle < torso_output.angle_min || angle > torso_output.angle_max)
          {
              NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, torso_output.angle_min,
                      torso_output.angle_max);
              continue;
          }

          //overwrite range at laserscan ray if new range is smaller
          int index = (angle - torso_output.angle_min) / torso_output.angle_increment;
          if (range < torso_output.ranges[index])
          {
              torso_output.ranges[index] = range;
          }

      }

    pub_.publish(legs_output);  // publish scan for legs
    torso_pub_.publish(torso_output); // publish scan_ubg for torso
  }

}

PLUGINLIB_DECLARE_CLASS(pointcloud_to_laserscan, PointCloudToLaserScanNodelet, pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet);
