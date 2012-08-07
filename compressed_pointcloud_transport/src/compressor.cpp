#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

//PCL specific includes
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <compressed_pointcloud_transport/CompressedPointCloud.h>

// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub;
pcl::octree::PointCloudCompression<PointT>* PointCloudEncoder;
pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);

// TODO: Could unsubscribe when no one is subscribed to the compressed output.
// Right now, though, we are assuming that local-loopback is low enough load that we don't care.
void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& rosCloud)
{
  
  if(pub.getNumSubscribers())
  {
    // Stringstream to store compressed point cloud
    std::stringstream compressedData;

    // Must convert from sensor_msg::PointCloud2 to pcl::PointCloud<PointT> for the encoder
    pcl::fromROSMsg(*rosCloud, *pclCloud);

    ROS_DEBUG_NAMED("compressor" ,"Compressing cloud with frame [%s] and stamp [%f]", pclCloud->header.frame_id.c_str(), pclCloud->header.stamp.toSec());

    // Compress the pointcloud
    PointCloudEncoder->encodePointCloud (pclCloud, compressedData);

    // Pack into a compressed message
    compressed_pointcloud_transport::CompressedPointCloud output;
    output.header = rosCloud->header;
    output.data = compressedData.str();
    pub.publish(output);

    int original_size = sizeof(rosCloud->data);
    int compressed_size = sizeof(output);
    ROS_DEBUG_NAMED("compressor", "Published cloud, original size %d bytes, compressed size %d bytes, %.3f of original.",
             original_size, compressed_size, (float)compressed_size/(float)original_size);
  }
  else
    ROS_DEBUG_NAMED("compressor" ,"Received input cloud but there are no subscribers; not publishing.");
}


pcl::octree::compression_Profiles_e getCompressionProfile(ros::NodeHandle nh)
{
  std::string compression_level;
  if(nh.getParam("compression_profile", compression_level))
  {
    if(compression_level == "LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR")        return pcl::octree::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    else if(compression_level == "LOW_RES_ONLINE_COMPRESSION_WITH_COLOR")      return pcl::octree::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR;
    else if(compression_level == "MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR")   return pcl::octree::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    else if(compression_level == "MED_RES_ONLINE_COMPRESSION_WITH_COLOR")      return pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
    else if(compression_level == "HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR")  return pcl::octree::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    else if(compression_level == "HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR")     return pcl::octree::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR;
    else if(compression_level == "LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR")  return pcl::octree::LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
    else if(compression_level == "LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR")     return pcl::octree::LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR;
    else if(compression_level == "MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR")  return pcl::octree::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
    else if(compression_level == "MED_RES_OFFLINE_COMPRESSION_WITH_COLOR")     return pcl::octree::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR;
    else if(compression_level == "HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR") return pcl::octree::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
    else if(compression_level == "HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR")    return pcl::octree::HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR;
    else return pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
  }
  else
  {
     return pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
  }
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "cloud_compressor");
  ros::NodeHandle nh, pnh("~");
  
  // Initialize encoder
  pcl::octree::compression_Profiles_e compressionProfile = getCompressionProfile(pnh);
  PointCloudEncoder = new pcl::octree::PointCloudCompression<PointT>(compressionProfile, false);

  //Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  //Create a ROS publisher for the output point cloud
  pub = nh.advertise<compressed_pointcloud_transport::CompressedPointCloud>("output", 10);

  //Spin
  ros::spin();
}
