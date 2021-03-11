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
#include <pcl/console/parse.h>

#include <boost/thread/thread.hpp>

ros::Publisher res_pub;

sensor_msgs::PointCloud2 res_border;
sensor_msgs::PointCloud2 res_veil;
sensor_msgs::PointCloud2 res_shadow;

float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

float angular_resolution_x = 0.5f, angular_resolution_y = angular_resolution_x;

bool live_update = true;



pcl::RangeImage& trans_cloud_to_image(const pcl::PointCloud<pcl::PointXYZ> point_cloud)
{
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity());
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f(point_cloud.sensor_origin_[0],
                                                              point_cloud.sensor_origin_[1],
                                                              point_cloud.sensor_origin_[2])) *
                                                                      Eigen::Affine3f (point_cloud.sensor_orientation_);
    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
                                     pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                     scene_sensor_pose, coordinate_frame, noise_level,min_range,border_size);
    return range_image;
}

void BorderExtractor(const pcl::RangeImage range_image)
{
    pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
    pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
            & veil_points = *veil_points_ptr,
            & shadow_points = *shadow_points_ptr;
    pcl::RangeImageBorderExtractor border_extractor (&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute(border_descriptions);

    for(int y = 0; y < (int)range_image.height; ++y)
    {
        for(int x = 0; x < (int)range_image.width; ++x)
        {
            if(border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                border_points.points.push_back(range_image.points[y*range_image.width + x]);

            if(border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
                veil_points.points.push_back(range_image.points[y*range_image.width + x]);

            if(border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
                shadow_points.points.push_back(range_image[y*range_image.width + x]);
        }
    }
    pcl::toROSMsg(border_points, res_border);
    pcl::toROSMsg(veil_points, res_veil);
    pcl::toROSMsg(shadow_points, res_shadow);
    res_border.header.frame_id = "kinect_frame_optical";
    res_border.header.stamp = ros::Time::now();
    res_pub.publish(res_border);
    //res_pub.publish(res_veil);
    //res_pub.publish(res_shadow);
}

void detectionCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_msg;
    pcl::fromROSMsg(*msg, pcl_msg);
    //pcl::RangeImage range_image = trans_cloud_to_image(*pcl_msg);

    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());  //传感器的位置

    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (pcl_msg.sensor_origin_[0],
                                                               pcl_msg.sensor_origin_[1],
                                                               pcl_msg.sensor_origin_[2])) *
                        Eigen::Affine3f (pcl_msg.sensor_orientation_);  //仿射变换矩阵

    float noise_level = 0.0;      //各种参数的设置
    float min_range = 0.0f;
    int border_size = 1;
    angular_resolution = pcl::deg2rad (angular_resolution);
    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    range_image.createFromPointCloud (pcl_msg, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                      scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    //range_image.integrateFarRanges (far_ranges);
    //if (setUnseenToMaxRange)
    //    range_image.setUnseenToMaxRange ();
    pcl::RangeImageBorderExtractor border_extractor (&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute (border_descriptions);     //提取边界计算描述子

    // -------------------------------------------------------
    // -----Show points in 3D viewer在3D 视口中显示点云-----
    // ----------------------------------------------------
    pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),  //物体边界
    veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),     //veil边界
    shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);   //阴影边界
    pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
            & veil_points = * veil_points_ptr,
            & shadow_points = *shadow_points_ptr;

    for (int y=0; y< (int)range_image.height; ++y)
    {
        for (int x=0; x< (int)range_image.width; ++x)
        {
            if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                border_points.points.push_back (range_image.points[y*range_image.width + x]);

            if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
                veil_points.points.push_back (range_image.points[y*range_image.width + x]);

            if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
                shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
        }
    }

    //   for(int i = 0; i < border_points.points.size(); ++i)
    //    std::cout << border_points.points[i] << " ";

    pcl::toROSMsg(border_points, res_border);
    pcl::toROSMsg(veil_points, res_veil);
    pcl::toROSMsg(shadow_points, res_shadow);

    res_border.header.frame_id = "kinect_frame_optical";
    res_border.header.stamp = ros::Time::now();

    res_pub.publish(res_border);
    //BorderExtractor(range_image);

    // --test--cout
    //for(int i = 0; i < pcl_msg->points.size(); ++i)
    //    std::cout << pcl_msg->points[i] << std::endl;
    //pcl::visualization::CloudViewer viewer_1 ("test");
    //viewer_1.showCloud(pcl_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh_;
    ros::Subscriber pointCloud_sub = nh_.subscribe("/kinect/depth/points",1000, detectionCallback);
    res_pub = nh_.advertise<sensor_msgs::PointCloud2>("res_cloud",1);
    ros::spin();
    return 0;
}
