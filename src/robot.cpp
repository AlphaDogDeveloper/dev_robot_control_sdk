/****************************************************************************** 
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
******************************************************************************/
#include "robot_control/robot.h"
#include "sensorimotor_interface.h"
#include <stdio.h>
#include <memory.h>
#ifdef __APPLE__
    // MacOS 
    #include <dispatch/dispatch.h>
#else
    // Linux 
    #include <sys/timerfd.h>
#endif

/**
 * Constructor method of class Robot.
 * @param config_path   YAML configure file path.
 */
Robot::Robot(const std::string &config_path) : node_("robot_control")
{
    /**
     * Load configure file
     */
    config_ = YAML::LoadFile(config_path);

    num_joints_ = config_["num_joints"].as<int>();

    memset(&imu_data_, 0, sizeof(imu_data_));
    actuator_data_.resize(num_joints_, {0});
    actuator_cmd_.resize(num_joints_, {0});
    actuator_directions_.resize(num_joints_);
    joint_zero_pos_.resize(num_joints_);

    /**
     * Initialize actuator directions
     */
    YAML::Node actuator_directions_config = config_["actuator_directions"];
    for (int i = 0; i < num_joints_; ++i)
    {
        actuator_directions_[i] = actuator_directions_config[i].as<int>();
    }

    /**
     * Initialize joint zero position
     */
    YAML::Node joint_zero_config = config_["joint_zero_pos"];
    for (int i = 0; i < num_joints_; ++i)
    {
        joint_zero_pos_[i] = DEGREE_TO_RAD(joint_zero_config[i].as<float>());
    }

    state_ = new RobotState();
    state_->joints.resize(num_joints_);
}

Robot::~Robot()
{
    delete state_;
    state_ = nullptr;
}

/**
 * Initialize the robot its various units.
 * @return 0 for success, others for failure.
 */
int Robot::initialize()
{
    int err;
    
    // Load configure parameters for initializing actuators
    int robot_type = config_["robot_type"].as<int>();
    std::string spi_path = config_["spi_dev"].as<std::string>();
    bool set_homing = config_["set_homing"].as<bool>();
    // Initialize actuators
    err = initialize_actuators(RobotType(robot_type), spi_path.c_str(), set_homing);
    if (err != 0)
    {
        printf("[Robot] Failed to initialize actuators '%s' (error:%d).\n", spi_path.c_str(), err);
        return -1;
    }

    // Load configure parameters for initializing IMU
    std::string imu_path = config_["imu_dev"].as<std::string>();
    // Initialize IMU
    err = initialize_imu(imu_path.c_str());
    if (err != 0)
    {
        printf("[Robot] Failed to initialize IMU '%s' (error:%d).\n", imu_path.c_str(), err);
        return -2;
    }

    // Load configure parameters for initializing BMS
    std::string battery_state_topic = config_["battery_state_topic"].as<std::string>();
    // Initialize IMU
    subscribers_.emplace_back(node_.subscribe(battery_state_topic, 3, &Robot::on_battery_state_callback, this));
    if (err != 0)
    {
        printf("[Robot] Failed to subscribe battery state topic '%s' (error:%d).\n", battery_state_topic.c_str(), err);
        return -3;
    }

    return err;
}

/**
 * Deinitialize the robot its various units.
 * @return 0 for success, others for failure.
 */
void Robot::deinitialize()
{
    deinitialize_actuators();
    deinitialize_imu();

    for (auto it = subscribers_.begin(); it != subscribers_.end(); it++)
    {
        it->shutdown();
    }
    subscribers_.clear();
}

/**
 * Enable the robot so we can control it to move.
 * @return 0 for success, others for failure.
 */
int Robot::enable()
{
    // Enable actuators
    return enable_actuators();
}

/**
 * Disable the robot to make it enter a low-power standby state immediately.
 * @return 0 for success, others for failure.
 */
int Robot::disable()
{
    // Disable actuators
    return disable_actuators();
}

/**
 * Update the current robot state including base pose, joints' position and so on.
 * @return 0 for success, others for failure.
 */
int Robot::update_state()
{
    int err;

    // Get actuators state
    err = get_actuators_data(actuator_data_.data(), num_joints_);
    if (err != 0)
    {
        printf("[Robot] Failed to get actuators data (error:%d).\n", err);
        return -1;
    }

    // Convert actuators state to joints state
    for (int i = 0; i < num_joints_; ++i)
    {
        state_->joints[i].pos = actuator_data_[i].pos / actuator_directions_[i] + joint_zero_pos_[i];
        state_->joints[i].vel = actuator_data_[i].vel / actuator_directions_[i];
        state_->joints[i].tau = actuator_data_[i].tau / actuator_directions_[i];
        state_->joints[i].online = actuator_data_[i].online;
        state_->joints[i].alarm = actuator_data_[i].alarm;
        state_->joints[i].mode = actuator_data_[i].mode;
        state_->joints[i].temperature = actuator_data_[i].temperature;
    }

    // Get IMU state
    err = get_imu_data(&imu_data_);
    if (err != 0)
    {
        printf("[Robot] Failed to get IMU data (error:%d).\n", err);
        return -2;
    }

    // Convert IMU state to base state
    state_->base.acc_x = imu_data_.acc_x;
    state_->base.acc_y = imu_data_.acc_y;
    state_->base.acc_z = imu_data_.acc_z;
    state_->base.gyro_x = imu_data_.gyro_x;
    state_->base.gyro_y = imu_data_.gyro_y;
    state_->base.gyro_z = imu_data_.gyro_z;
    state_->base.pitch = imu_data_.pitch;
    state_->base.roll = imu_data_.roll;
    state_->base.yaw = imu_data_.yaw;
    state_->base.quaternion_w = imu_data_.quaternion_w;
    state_->base.quaternion_x = imu_data_.quaternion_x;
    state_->base.quaternion_y = imu_data_.quaternion_y;
    state_->base.quaternion_z = imu_data_.quaternion_z;

    // Copy battery state
    battery_state_mutex_.lock();
    if (state_->battery.header.stamp != battery_state_.header.stamp)
    {
        state_->battery = battery_state_;
    }
    battery_state_mutex_.unlock();

    return 0;
}

/**
 * Callback of ROS subscriber to battery state topic.
 */
void Robot::on_battery_state_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    battery_state_mutex_.lock();
    battery_state_ = *msg;
    battery_state_mutex_.unlock();
}

/**
 * Send control command to joints.
 * @return 0 for success, others for failure.
 */
int Robot::send_command(const RobotCommand &command)
{
    int err;

    if (command.joints.size() != num_joints_)
    {
        return -1;
    }

    // Convert joints command to actuators command
    for (int i = 0; i < num_joints_; ++i)
    {
        actuator_cmd_[i].kp = command.joints[i].kp;
        actuator_cmd_[i].kd = command.joints[i].kd;
        actuator_cmd_[i].pos = (command.joints[i].pos - joint_zero_pos_[i]) * actuator_directions_[i];
        actuator_cmd_[i].vel = command.joints[i].vel * actuator_directions_[i];
        actuator_cmd_[i].tau = command.joints[i].tau * actuator_directions_[i];
    }

    // Send cmd
    err = send_command_to_actuators_async(actuator_cmd_.data(), num_joints_);
    if (err != 0)
    {
        printf("[Robot] Failed to send command.\n");
    }

    return 0;
}
