/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Michal Drwiega (drwiega.michal@gmail.com)
 * Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 * Based on the code of "nav_layer_from_points" published by:
 * Michal Drwiega <drwiega.michal AT gmail DOT com>
 * 
 * Github repo of original code:
 * https://github.com/mdrwiega/depth_nav_tools.git
 * 
 * This version modified by:
 * andres.arboleda AT gmail DOT com, (may/2020)
 */


#ifndef COSTMAP_LAYER_H
#define COSTMAP_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/layer.h>
#include <pluginlib/class_list_macros.h>

#include <depth_nav_msgs/Point32List.h>
#include <geometry_msgs/PointStamped.h>

#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <math.h>
#include <algorithm>

#include <angles/angles.h>

#include <dynamic_reconfigure/server.h>
#include <vva_cliff_detector_layer/VVACliffDetectorLayerConfig.h>


namespace vva_cliff_detector_layer
{
class VVACliffDetectorLayer : public costmap_2d::Layer
{
public:
  VVACliffDetectorLayer() { layered_costmap_ = NULL; }

  virtual void onInitialize();

  virtual void updateBounds( double origin_x, double origin_y, double origin_yaw,
                             double* min_x, double* min_y, double* max_x, double* max_y);

  virtual void updateCosts( costmap_2d::Costmap2D& master_grid,
                            int min_i, int min_j, int max_i, int max_j);

  void updateBoundsFromPoints( double* min_x, double* min_y,
                               double* max_x, double* max_y);

  bool isDiscretized() { return false; }

protected:

  void pointsCallback(const depth_nav_msgs::Point32List& points);

  /**
   * @brief clearTransformedPoints clears points from list transformed_points_ after some time
   */
  void clearTransformedPoints();

  void configure(VVACliffDetectorLayerConfig &config, uint32_t level);

protected: // Protected fields
  ros::Subscriber sub_points_;               ///< Subscriber for points
  depth_nav_msgs::Point32List points_list_;  ///< List of received points

  tf::TransformListener tf_;

  std::list<geometry_msgs::PointStamped> transformed_points_;

  // After this time points will be delete
  ros::Duration points_keep_time_;

  boost::recursive_mutex lock_;
  bool first_time_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

private:
  //-----------------------------------------------------------------------------------------------
  // ROS parameters configurated with config file or dynamic_reconfigure
  double point_radius_;
  double robot_radius_;

  //-----------------------------------------------------------------------------------------------

  dynamic_reconfigure::Server<VVACliffDetectorLayerConfig>* rec_server_;
  dynamic_reconfigure::Server<VVACliffDetectorLayerConfig>::CallbackType f_;

};
} // end of namespace


#endif

