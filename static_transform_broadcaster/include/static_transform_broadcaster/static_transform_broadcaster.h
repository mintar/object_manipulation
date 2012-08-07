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
#ifndef _STATIC_TRANSFORM_BROADCASTER_H_
#define _STATIC_TRANSFORM_BROADCASTER_H_

#include <map>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace static_transform_broadcaster {

//! Continuously re-publishes passed in transforms
/*! Used to keep in the system transforms that do not change often, but nobody wants to keep
  re-publishing. Just pass a transform to this class and it will keep republishing it for you
  until you remove it.
  
  Transforms are uniquely identified by their child_frame_id. If a passed in transform has the same
  child_frame_id as an existing one, the old one is replaced by the new one.

  The re-publishing is done at a constant rate which can be modified.
 */
class StaticTransformBroadcaster
{
private:
  //! The broadcaster used 
  tf::TransformBroadcaster broadcaster_;

  //! The internal list of transforms to be broadcast
  std::map<std::string, geometry_msgs::TransformStamped> transforms_;
  
  //! The broadcast rate, in seconds
  double publishing_rate_;

  //! Internal flag showing the instance is being destructed
  bool finished_;

  //! The thread used for publishing
  boost::thread *publishing_thread_;

  //! Mutex for access control
  boost::mutex mutex_;

  //! Does all the publishing work. Broadcasts all transforms with a time stamp of ros::Time::now()
  //! then sleeps for the duration equal to the publishing rate
  void publishingThread()
  {
    while (!finished_)
    {
      ros::Duration(publishing_rate_).sleep();
      std::map<std::string, geometry_msgs::TransformStamped>::iterator it;
      mutex_.lock();
      for (it = transforms_.begin(); it!=transforms_.end(); it++)
      {
        geometry_msgs::TransformStamped trans = it->second;
        trans.header.stamp = ros::Time::now();
        broadcaster_.sendTransform(trans);
      }
      mutex_.unlock();
    }
  }
  
public:
  //! Starts the publishing thread
  StaticTransformBroadcaster() : publishing_rate_(0.1), finished_(false) 
  {
    publishing_thread_ = new boost::thread(boost::bind(&StaticTransformBroadcaster::publishingThread, this)); 
  }

  //! Stops and joins the publishing thread
  ~StaticTransformBroadcaster() 
  {
    finished_ = true;
    publishing_thread_->join();
    delete publishing_thread_;
  }

  //! Add a transform to the list or change an existing one
  /*! If a transform already exists with the same child_frame_id, it is replaced by this new transform */
  void setTransform(geometry_msgs::TransformStamped transform)
  {
    mutex_.lock();
    transforms_[transform.child_frame_id] = transform;
    mutex_.unlock();
  }

  //! Removes a transform from the list, based on teh child_frame_id
  void removeTransform(std::string child_frame_id)
  {
    mutex_.lock();
    std::map<std::string, geometry_msgs::TransformStamped>::iterator it = transforms_.find(child_frame_id);    
    if (it != transforms_.end())
    {
      transforms_.erase(it);
    }
    else
    {
      ROS_WARN("Static transform broadcaster: attempt to erase non-existent transform to frame %s",
               child_frame_id.c_str());
    }
    mutex_.unlock();
  }
  
  //! Removes all transforms from the list
  void removeAllTransforms()
  {
    transforms_.clear();
  }

  //! Sets the publishing rate. If passed value is negative, 0.0 is used instead
  void setPublishingRate(double rate)
  {
    publishing_rate_ = std::max(0.0, rate);
  }

  double getPublishingRate() const 
  {
    return publishing_rate_;
  }
};

} //namespace

#endif

