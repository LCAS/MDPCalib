#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <iostream>

#include "pose_synchronizer/pose_synchronizer.h"

using pose_synchronizer::PoseSynchronizer;

int main(int argc, char* argv[]) {
    std::cout << "Starting pose synchronizer" << std::endl;

    ros::init(argc, argv, "pose_synchronizer");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    bool disable_pose_synchronizer = false;
    private_nh.param<bool>("disable_pose_synchronizer", disable_pose_synchronizer, false);
    const std::string calibration_options_path = ros::package::getPath("calib_cfg") + "/config/config.yaml";

    PoseSynchronizer pose_synchronizer(&nh, calibration_options_path, disable_pose_synchronizer);

    std::string camera_pose_topic;
    std::string camera_image_topic;
    std::string lidar_odometry_topic;
    std::string lidar_pointcloud_topic;
    std::string initial_transform_meta_topic;
    private_nh.param<std::string>("camera_pose_topic", camera_pose_topic, "/orb_slam3/camera_pose");
    private_nh.param<std::string>("camera_image_topic", camera_image_topic, "/camera_undistorted/image");
    private_nh.param<std::string>("lidar_odometry_topic", lidar_odometry_topic, "/odom");
    private_nh.param<std::string>("lidar_pointcloud_topic", lidar_pointcloud_topic, "/cloud_registered_body");
    private_nh.param<std::string>("initial_transform_meta_topic", initial_transform_meta_topic,
                                  "/optimizer/initial_transform_meta");

    // Subscribe to visual odometry and images
    message_filters::Subscriber<geometry_msgs::PoseStamped> camera_odometry_subscriber(nh, camera_pose_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> camera_image_subscriber(nh, camera_image_topic, 1);
    typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseStamped, sensor_msgs::Image>
        camera_sync_policy;
    message_filters::Synchronizer<camera_sync_policy> camera_sync(camera_sync_policy(10), camera_odometry_subscriber,
                                                                  camera_image_subscriber);
    camera_sync.registerCallback(boost::bind(&PoseSynchronizer::CameraCallback, &pose_synchronizer, _1, _2));

    // Subscribe to LiDAR odometry and point clouds
    message_filters::Subscriber<nav_msgs::Odometry> lidar_odometry_subscriber(nh, lidar_odometry_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_pointcloud_subscriber(nh, lidar_pointcloud_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2>
        lidar_sync_policy;
    message_filters::Synchronizer<lidar_sync_policy> lidar_sync(lidar_sync_policy(10), lidar_odometry_subscriber,
                                                                lidar_pointcloud_subscriber);
    lidar_sync.registerCallback(boost::bind(&PoseSynchronizer::LidarCallback, &pose_synchronizer, _1, _2));

    // Subscribe to initial transform from the optimizer to terminate this node
    ros::Subscriber initial_transform_meta_subscriber = nh.subscribe(initial_transform_meta_topic, 10,
                                                                     &PoseSynchronizer::TerminateCallback,
                                                                     &pose_synchronizer);

    ros::spin();

    return 0;
}
