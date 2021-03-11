//
// Created by xcy on 2021/3/9.
//

#include "pointcloud.h"

void PointCloud::detectionCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_msg);
    pcl::RangeImage range_image = trans_cloud_to_image(*pcl_msg);
    //for(int i = 0; i < range_image.points.size(); ++i)
        //std::cout << range_image.points[i];
    //res_border.sensor_orientation_ = msg->sensor_orientation_;
    //res_border.sensor_origin_ = msg->sensor_origin_;
    //res_border.points.resize(msg->points.size ());
    BorderExtractor(range_image);

    // --test--cout
    //for(int i = 0; i < pcl_msg->points.size(); ++i)
    //    std::cout << pcl_msg->points[i] << std::endl;
    //pcl::visualization::CloudViewer viewer_1 ("test");
    //viewer_1.showCloud(pcl_msg);
}

pcl::RangeImage& PointCloud::trans_cloud_to_image(const pcl::PointCloud<pcl::PointXYZ> point_cloud)
{
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    float angular_resolution_x;
    float angular_resolution_y;
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

void PointCloud::BorderExtractor(const pcl::RangeImage range_image)
{
    pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
    pcl::PointCloud<pcl::PointWithRange>::Ptr veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
    pcl::PointCloud<pcl::PointWithRange>::Ptr shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);

    pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr;
    pcl::PointCloud<pcl::PointWithRange>& veil_points = *veil_points_ptr;
    pcl::PointCloud<pcl::PointWithRange>& shadow_points = *shadow_points_ptr;
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
    std::cout << border_points.points.size();
    for(int i = 0; i < border_points.points.size(); ++i)
         std::cout << border_points.points[i] << " " << endl;
    pcl::toROSMsg(border_points, res_border);
    pcl::toROSMsg(veil_points, res_veil);
    pcl::toROSMsg(shadow_points, res_shadow);
    res_border.header.frame_id = "kinect_frame_optical";
    res_border.header.stamp = ros::Time::now();
    res_pub.publish(res_border);
    //res_pub.publish(res_veil);
    //res_pub.publish(res_shadow);
}

