/*
 * File: bag_converter.cpp
 * Description: Offline tool to convert hesai_ros_driver/UdpFrame packets
 *              from a rosbag into sensor_msgs/PointCloud2 messages,
 *              writing the result to a new bag.
 *
 * Usage: rosrun hesai_ros_driver bag_converter input.bag output.bag
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "hesai_ros_driver/UdpFrame.h"
#include "hesai_ros_driver/UdpPacket.h"
#include "source_drive_common.hpp"
#include "hesai_lidar_sdk.hpp"
#include "Version.h"

#include <atomic>
#include <thread>
#include <mutex>
#include <signal.h>

using namespace hesai::lidar;

static std::atomic<bool> g_shutdown{false};

static void sigHandler(int) {
  g_shutdown = true;
}

static sensor_msgs::PointCloud2 toRosMsg(
    const LidarDecodedFrame<LidarPointXYZIRT>& frame,
    const std::string& frame_id) {
  sensor_msgs::PointCloud2 ros_msg;

  int fields = 6;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);
  ros_msg.width = frame.points_num;
  ros_msg.height = 1;

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = false;
  ros_msg.data.resize(frame.points_num * ros_msg.point_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp(ros_msg, "timestamp");
  for (size_t i = 0; i < frame.points_num; i++) {
    LidarPointXYZIRT point = frame.points[i];
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    *iter_intensity = point.intensity;
    *iter_ring = point.ring;
    *iter_timestamp = point.timestamp;
    ++iter_x; ++iter_y; ++iter_z;
    ++iter_intensity; ++iter_ring; ++iter_timestamp;
  }

  int64_t sec = static_cast<int64_t>(frame.points[0].timestamp);
  if (sec <= std::numeric_limits<int32_t>::max()) {
    ros_msg.header.stamp = ros::Time().fromSec(frame.points[0].timestamp);
  }
  ros_msg.header.frame_id = frame_id;
  return ros_msg;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    printf("Usage: rosrun hesai_ros_driver bag_converter <input.bag> <output.bag>\n");
    return 1;
  }

  std::string input_bag_path = argv[1];
  std::string output_bag_path = argv[2];

  signal(SIGINT, sigHandler);
  ros::init(argc, argv, "bag_converter", ros::init_options::NoSigintHandler);

  // Load config
  std::string config_path;
#ifdef RUN_IN_ROS_WORKSPACE
  config_path = ros::package::getPath("hesai_ros_driver");
#else
  config_path = (std::string)PROJECT_PATH;
#endif
  config_path += "/config/config.yaml";

  YAML::Node config = YAML::LoadFile(config_path);
  YAML::Node lidar_config = YamlSubNodeAbort(config, "lidar");
  YAML::Node sensor_config = lidar_config[0];

  // Parse driver params
  DriverParam driver_param;
  DriveYamlParam yaml_param;
  yaml_param.GetDriveYamlParam(sensor_config, driver_param);

  std::string frame_id = driver_param.input_param.frame_id;
  std::string packet_topic = driver_param.input_param.ros_recv_packet_topic;
  std::string cloud_topic = driver_param.input_param.ros_send_point_topic;

  // Force rosbag packet mode
  driver_param.input_param.source_type = DATA_FROM_ROS_PACKET;
  driver_param.decoder_param.enable_udp_thread = false;
  driver_param.decoder_param.enable_parser_thread = true;

  printf("bag_converter: input  = %s\n", input_bag_path.c_str());
  printf("bag_converter: output = %s\n", output_bag_path.c_str());
  printf("bag_converter: packet_topic = %s\n", packet_topic.c_str());
  printf("bag_converter: cloud_topic  = %s\n", cloud_topic.c_str());

  // Open output bag
  rosbag::Bag out_bag;
  out_bag.open(output_bag_path, rosbag::bagmode::Write);
  std::mutex out_bag_mutex;

  // Create SDK and register pointcloud callback
  std::atomic<uint32_t> frame_count{0};
  auto driver_ptr = std::make_shared<HesaiLidarSdk<LidarPointXYZIRT>>();
  driver_ptr->RegRecvCallback(
      [&](const LidarDecodedFrame<LidarPointXYZIRT>& frame) {
        sensor_msgs::PointCloud2 cloud_msg = toRosMsg(frame, frame_id);
        printf("frame:%d points:%u packet:%d start time:%lf end time:%lf\n",
               frame.frame_index, frame.points_num, frame.packet_num,
               frame.points[0].timestamp,
               frame.points[frame.points_num - 1].timestamp);
        {
          std::lock_guard<std::mutex> lock(out_bag_mutex);
          out_bag.write(cloud_topic, cloud_msg.header.stamp, cloud_msg);
        }
        frame_count++;
      });

  if (!driver_ptr->Init(driver_param)) {
    printf("bag_converter: Driver Init failed\n");
    return 1;
  }

  // Open input bag and start reader thread.
  // The reader must run before Start() so the SDK init thread can
  // auto-detect the lidar type from the first packet.
  std::atomic<bool> reader_done{false};
  std::thread reader_thread([&]() {
    rosbag::Bag in_bag;
    in_bag.open(input_bag_path, rosbag::bagmode::Read);
    rosbag::View view(in_bag);

    uint64_t packet_count = 0;
    for (const rosbag::MessageInstance& m : view) {
      if (g_shutdown) break;

      if (m.getTopic() == packet_topic) {
        hesai_ros_driver::UdpFrame::ConstPtr udp_frame =
            m.instantiate<hesai_ros_driver::UdpFrame>();
        if (udp_frame) {
          for (size_t i = 0; i < udp_frame->packets.size(); i++) {
            // Wait if the ring buffer is full
            while (driver_ptr->OriginPacketIsBufferFull() && !g_shutdown) {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if (g_shutdown) break;
            driver_ptr->lidar_ptr_->origin_packets_buffer_.emplace_back(
                &udp_frame->packets[i].data[0], udp_frame->packets[i].size);
            packet_count++;
          }
        }
      } else {
        // Copy non-packet messages directly to output
        std::lock_guard<std::mutex> lock(out_bag_mutex);
        out_bag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
      }
    }

    in_bag.close();
    printf("bag_converter: finished reading input bag (%lu packets)\n", packet_count);
    reader_done = true;
  });

  // Start SDK processing (blocks until init completes, then launches Run thread)
  driver_ptr->Start();

  // Wait for reader to finish
  reader_thread.join();

  // Wait for the SDK to drain remaining packets
  printf("bag_converter: waiting for processing to complete...\n");
  while (!g_shutdown) {
    if (driver_ptr->lidar_ptr_->origin_packets_buffer_.empty()) {
      // Buffer is empty; give the Run/parser threads time to finish
      // processing the last frame
      std::this_thread::sleep_for(std::chrono::seconds(2));
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  driver_ptr->Stop();
  out_bag.close();

  printf("bag_converter: done. wrote %u pointcloud frames to %s\n",
         frame_count.load(), output_bag_path.c_str());
  return 0;
}
