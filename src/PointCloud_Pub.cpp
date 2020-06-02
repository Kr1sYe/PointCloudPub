#include <ros/ros.h>
#include <iostream> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>      //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h>    //PCL对各种格式的点的支持头文件

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    // 定义发布的消息
    sensor_msgs::PointCloud2 output;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>); // 创建点云（指针）

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        PCL_ERROR ("Couldn't read file \n"); //文件不存在时，返回错误，终止程序。
        return (-1);
    }
    std::cout << "Loaded "
                << cloud->width * cloud->height
                << " data points from pcd file with the following fields: "
                << std::endl;

    // for (size_t i = 0; i < cloud->points.size (); ++i)   //显示所有的点
    for (size_t i = 0; i < 5; ++i)                          // 为了方便观察，只显示前5个点
    {                         
        std::cout << "    " << cloud->points[i].x
                << " "    << cloud->points[i].y
                << " "    << cloud->points[i].z 
                << " "    << int(cloud->points[i].r)
                << " "    << int(cloud->points[i].g) 
                << " "    << int(cloud->points[i].b)
                << std::endl;
    }

    //Convert the cloud to ROS message
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";
    std::cout << output.data.size() << std::endl;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();

        ROS_INFO("Publishing PCD PointCloud");
    }

    ros::shutdown();

}