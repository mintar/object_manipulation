/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#include <ros/ros.h>

#include "static_transform_broadcaster/SetStaticTransform.h"
#include "static_transform_broadcaster/RemoveStaticTransforms.h"

#include "static_transform_broadcaster/static_transform_broadcaster.h"

namespace static_transform_broadcaster {

static std::string set_transform_service_name = "set_static_transform";
static std::string remove_transform_service_name = "remove_static_transform";

class StaticTransformBroadcasterNode
{
private:
  StaticTransformBroadcaster broadcaster_;

  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Node handle in the root namespace
  ros::NodeHandle root_nh_;
  
  ros::ServiceServer set_transform_srv_;
  ros::ServiceServer remove_transforms_srv_;
  
  bool setTransformCB(SetStaticTransform::Request &request, SetStaticTransform::Response &response)
  {
    broadcaster_.setTransform(request.transform);
    return true;
  }

  bool removeTransformsCB(RemoveStaticTransforms::Request &request, RemoveStaticTransforms::Response &response)
  {
    if (request.remove_all_transforms)
    {
      broadcaster_.removeAllTransforms();
      return true;
    }
    for (size_t i=0; i<request.child_frame_ids.size(); i++)
    {
      broadcaster_.removeTransform(request.child_frame_ids[i]);
    }
    return true;
  }

public:
  StaticTransformBroadcasterNode() : priv_nh_("~"), root_nh_("")
  {
    set_transform_srv_ = priv_nh_.advertiseService(set_transform_service_name, 
                                                   &StaticTransformBroadcasterNode::setTransformCB, this);
    remove_transforms_srv_ = priv_nh_.advertiseService(remove_transform_service_name, 
                                                       &StaticTransformBroadcasterNode::removeTransformsCB, this);
  }

  ~StaticTransformBroadcasterNode() {}
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_transform_broadcaster");
  ros::NodeHandle nh;
  static_transform_broadcaster::StaticTransformBroadcasterNode node;
  ros::spin();
  return 0;
}
