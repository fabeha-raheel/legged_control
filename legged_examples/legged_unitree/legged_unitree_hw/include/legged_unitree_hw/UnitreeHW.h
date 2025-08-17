//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include <legged_hw/LeggedHW.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>       // For subscribing to joint states
#include <std_msgs/Float64MultiArray.h>   // For publishing commands
#include <std_msgs/Int16MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/safety.h"
#include "unitree_legged_sdk_3_3_1/udp.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/safety.h"
#include "unitree_legged_sdk_3_8_0/udp.h"
#endif

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct UnitreeMotorData {
  double pos_, vel_, tau_;                 // Feedback from hardware
  double posDes_, velDes_, kp_, kd_, ff_;  // Commanded values
};

struct UnitreeImuData {
  double ori_[4];
  double oriCov_[9];
  double angularVel_[3];
  double angularVelCov_[9];
  double linearAcc_[3];
  double linearAccCov_[9];
};

class UnitreeHW : public LeggedHW {
 public:
  UnitreeHW() = default;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  void updateJoystick(const ros::Time& time);
  void updateContact(const ros::Time& time);

  // IMU callback
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  // Joint state callback
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

 private:
  bool setupJoints();
  bool setupImu();
  bool setupContactSensor(ros::NodeHandle& nh);

  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState lowState_{};
  UNITREE_LEGGED_SDK::LowCmd lowCmd_{};

  UnitreeMotorData jointData_[12]{};
  UnitreeImuData imuData_{};
  bool contactState_[4]{};
  std::vector<std::string> jointNames_;

  int powerLimit_{};
  int contactThreshold_{};

  ros::Publisher joyPublisher_;
  ros::Publisher contactPublisher_;
  ros::Publisher command_pub_;        // Publisher for commands
  ros::Subscriber joint_state_sub_;   // Subscriber for joint states
  ros::Subscriber imu_sub_;

  ros::Time lastJoyPub_, lastContactPub_;
};

}  // namespace legged
