//
// Created by xcy on 2021/3/9.
//

#include "pointcloud.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tester");

    PointCloud my_pub;

    ros::spin();
    return 0;
}
