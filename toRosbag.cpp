#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// PCD 파일을 ROS 메시지로 변환하는 함수
sensor_msgs::PointCloud2 convertToROSMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Time & timestamp)
{
  sensor_msgs::PointCloud2 rosMsg;
  pcl::toROSMsg(*cloud, rosMsg);
  rosMsg.header.frame_id = "/scan";
  rosMsg.header.stamp = timestamp;
  return rosMsg;
}

int main(int argc, char** argv) {
  // ROS 초기화
  ros::init(argc, argv, "pcd_to_rosbag");
  ros::NodeHandle nh;

  // ROSBAG 생성
  std::string rosbagFile = "output/robot1.bag";
  rosbag::Bag bag;
  bag.open(rosbagFile, rosbag::bagmode::Write);

  // PCD 파일 변환
  ros::Time timestamp = ros::Time::now();  // 현재 타임스탬프
  for (int i = 0; i < 2213; ++i) {
    std::string pcdFile = "raw_data/test/test_pcd" + std::to_string(i) + ".pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFile, *cloud) == -1) {
      std::cerr << "Failed to load PCD file: " << pcdFile << std::endl;
      continue;
    }

    // PCD 파일을 ROS 메시지로 변환하여 ROSBAG에 저장
    sensor_msgs::PointCloud2 rosMsg = convertToROSMsg(cloud, timestamp);
    bag.write("/point_cloud", rosMsg.header.stamp, rosMsg);
    timestamp += ros::Duration(0.1);
  }
  // ROSBAG 닫기
  bag.close();
  std::cout << "ROSBAG conversion completed." << std::endl;
  return 0;
}
