


#include "ros/ros.h"
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/features/normal_3d.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include "pcl/range_image/range_image.h"
#include "pcl/features/range_image_border_extractor.h"

#include <boost/thread/thread.hpp>

#ifndef SRC_POINTCLOUD_H
#define SRC_POINTCLOUD_H

class PointCloud
{
public:
    ros::Publisher res_pub;
    ros::Subscriber pointCloud_sub;
    ros::NodeHandle nh_;
    void detectionCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
   
    PointCloud(): angular_resolution_x(0.5f), angular_resolution_y(0.5f),
    coordinate_frame(pcl::RangeImage::CAMERA_FRAME), live_update(true){
        pointCloud_sub = nh_.subscribe("/kinect/depth/points",1000, &PointCloud::detectionCallback, this);
        res_pub = nh_.advertise<sensor_msgs::PointCloud2>("res_cloud",10);
    }
    
    pcl::RangeImage& trans_cloud_to_image(const pcl::PointCloud<pcl::PointXYZ> point_cloud);
    void BorderExtractor(const pcl::RangeImage range_image);

private:
    sensor_msgs::PointCloud2 res_border;
    sensor_msgs::PointCloud2 res_veil;
    sensor_msgs::PointCloud2 res_shadow;

    float angular_resolution_x;
    float angular_resolution_y;

    pcl::RangeImage::CoordinateFrame coordinate_frame;

    bool live_update;
};


#endif
