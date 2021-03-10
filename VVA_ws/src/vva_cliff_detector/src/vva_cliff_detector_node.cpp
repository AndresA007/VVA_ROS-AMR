/* 
 * Copyright (c) 2015, Michal Drwiega <drwiega.michal AT gmail DOT com>
 * Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the <COPYRIGHT HOLDER>.
 * 4. Neither the name of the <COPYRIGHT HOLDER> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Based on the code of "cliff_detector" published by:
 * Michal Drwiega <drwiega.michal AT gmail DOT com>
 * 
 * Github repo of original code:
 * https://github.com/mdrwiega/depth_nav_tools.git
 * 
 * This version modified by:
 * andres.arboleda AT gmail DOT com, (may/2020)
 */




#include <vva_cliff_detector/vva_cliff_detector_node.h>

namespace vva_cliff_detector {

VVACliffDetectorNode::VVACliffDetectorNode(ros::NodeHandle& n, ros::NodeHandle& pnh):
  node_rate_hz_(2), pnh_(pnh), it_(n), reconf_srv_(pnh)
{
  boost::mutex::scoped_lock lock(connection_mutex_);

  // Set callback for dynamic reconfigure server
  reconf_srv_.setCallback(boost::bind(&VVACliffDetectorNode::reconfigureCb, this, _1, _2));

  // New depth image publisher
  //~ pub_ = it_.advertise("cliff_detector/depth", 1,
                       //~ boost::bind(&CliffDetectorNode::connectCb, this),
                       //~ boost::bind(&CliffDetectorNode::disconnectCb, this));
  pub_ = it_.advertise("cliff_detector/depth", 1);

  // Publisher for stairs points msg
  pub_points_ = n.advertise<depth_nav_msgs::Point32List>("cliff_detector/points", 2);
  
  ROS_INFO("Connecting to depth topic.");
  image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
  sub_ = it_.subscribeCamera("image", 1, &VVACliffDetectorNode::depthCb, this, hints);
}

VVACliffDetectorNode::~VVACliffDetectorNode()
{
  sub_.shutdown();
}

void VVACliffDetectorNode::setNodeRate(const unsigned int rate)
{
  if (rate <= 30)
    node_rate_hz_ = rate;
  else
    node_rate_hz_ = 30;
}

unsigned int VVACliffDetectorNode::getNodeRate()
{
  return node_rate_hz_;
}

void VVACliffDetectorNode::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  try
  {
    // Run cliff detector based on depth image
    detector_.detectCliff(depth_msg, info_msg);

    // Publish stairs points msg
    pub_points_.publish(detector_.stairs_points_msg_);

    // Publishes new depth image with added cliff
    if (detector_.getPublishDepthEnable() && pub_.getNumSubscribers() > 0)
      pub_.publish(detector_.new_depth_msg_);
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_THROTTLE(1.0, "Could not perform stairs detection: %s", e.what());
  }
}

//~ void CliffDetectorNode::connectCb()
//~ {
  //~ boost::mutex::scoped_lock lock(connection_mutex_);
  //~ if (!sub_ && pub_.getNumSubscribers() > 0)
  //~ {
    //~ ROS_INFO("Connecting to depth topic.");
    //~ image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    //~ sub_ = it_.subscribeCamera("image", 1, &CliffDetectorNode::depthCb, this, hints);
  //~ }
//~ }

//~ void CliffDetectorNode::disconnectCb()
//~ {
  //~ boost::mutex::scoped_lock lock(connection_mutex_);
  //~ if (pub_.getNumSubscribers() == 0)
  //~ {
    //~ ROS_INFO("Unsubscribing from depth topic.");
    //~ sub_.shutdown();
  //~ }
//~ }

void VVACliffDetectorNode::reconfigureCb(
    vva_cliff_detector::VVACliffDetectorConfig& config, uint32_t level)
{
  node_rate_hz_ = (unsigned int) config.rate;

  detector_.setRangeLimits(config.range_min, config.range_max);
  detector_.setSensorMountHeight(config.sensor_mount_height);
  detector_.setSensorTiltAngle(config.sensor_tilt_angle);
  detector_.setPublishDepthEnable(config.publish_depth);
  detector_.setCamModelUpdate(config.cam_model_update);

  detector_.setUsedDepthHeight((unsigned int)config.used_depth_height);
  detector_.setBlockSize(config.block_size);
  detector_.setBlockPointsThresh(config.block_points_thresh);
  detector_.setDepthImgStepRow(config.depth_img_step_row);
  detector_.setDepthImgStepCol(config.depth_img_step_col);
  detector_.setGroundMargin(config.ground_margin);

  detector_.setParametersConfigurated(false);
}

}
