#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "imu_tk/io_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("imu_calib_node");
    std::string bag_file = node->declare_parameter<std::string>("ros2_bag_path");
    std::string imu_topic = node->declare_parameter<std::string>("imu_topic");
    
    std::vector<imu_tk::TriadData> acc_data, gyro_data;

    rosbag2_storage::StorageOptions storage_options;
    rosbag2_cpp::ConverterOptions converter_options;
    storage_options.uri = bag_file;
    storage_options.storage_id = "sqlite3";
    converter_options.input_serialization_format = "cdr";

    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(storage_options, converter_options);

    rosbag2_storage::StorageFilter storage_filter;
    storage_filter.topics = {imu_topic};
    reader.set_filter(storage_filter);

    while(reader.has_next()){
        auto bag_message = reader.read_next();

        sensor_msgs::msg::Imu imu_msg;
        rclcpp::Serialization<sensor_msgs::msg::Imu> serializetion;
        rclcpp::SerializedMessage extracted_msg(*bag_message->serialized_data);
        serializetion.deserialize_message(&extracted_msg, &imu_msg);
        double time_stamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;
        acc_data.push_back(imu_tk::TriadData(time_stamp, imu_msg.linear_acceleration.x, 
                        imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z));
        gyro_data.push_back(imu_tk::TriadData(time_stamp, imu_msg.angular_velocity.x,
                        imu_msg.angular_velocity.y, imu_msg.angular_velocity.z));
    }
    RCLCPP_INFO(node->get_logger(), "Read %d tuples", acc_data.size());
    imu_tk::CalibratedTriad init_acc_calib, init_gyro_calib;
    init_acc_calib.setBias(Eigen::Vector3d::Zero());
    init_gyro_calib.setScale(Eigen::Vector3d::Ones());

    imu_tk::MultiPosCalibration calib;

    calib.setInitStaticIntervalDuration(50.0);
    calib.setInitAccCalibration(init_acc_calib);
    calib.setInitGyroCalibration(init_gyro_calib);
    RCLCPP_INFO(node->get_logger(), "Init complete");
    calib.setGravityMagnitude(9.789);
    calib.enableVerboseOutput(false);
    calib.enableAccUseMeans(false);
    calib.calibrateAccGyro(acc_data, gyro_data);
    RCLCPP_INFO(node->get_logger(), "Calibration complete");
    calib.getAccCalib().save("imu_calib.yaml");
    calib.getGyroCalib().save("imu_calib.yaml");


    return 0;
}