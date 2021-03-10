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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vva_cliff_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("Start vva_cliff_detector");
  
  vva_cliff_detector::VVACliffDetectorNode detector(nh, pnh);
  
  while (ros::ok())
  {
    ros::Rate rate(detector.getNodeRate());
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
