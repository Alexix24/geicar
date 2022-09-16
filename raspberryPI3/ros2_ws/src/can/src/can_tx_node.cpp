#include <stdint.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/steering_calibration.hpp"

#include "../include/can/can.h"

using std::placeholders::_1;



class can_tx : public rclcpp::Node {
public:
  can_tx()
  : Node("can_tx_node")
  {
    if(initCommunication()==0) {
      subscription_motors_order_ = this->create_subscription<interfaces::msg::MotorsOrder>(
      "send_motors_order", 10, std::bind(&can_tx::sendMotorsOrderCallback, this, _1));

      subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
      "steering_calibration", 10, std::bind(&can_tx::sendCalibrationRequestCallback, this, _1));
    }
  }

  int closeCommunication(){

    if (s==0 && close(s) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Close communication failed - Socket close error");
        return 1;
    }
    RCLCPP_INFO(this->get_logger(), "CAN communication closed");
    return 0;
  }


private:
  int initCommunication(){
        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "CAN initialization failed - Socket creation error");
            return 1;
        }

        strcpy(ifr.ifr_name, "can0" );
        ioctl(s, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "CAN initialization failed - Bind error");
            return 1;
        }

        frame.can_dlc = 8;

        RCLCPP_INFO(this->get_logger(), "CAN communication initialized");

      return 0;
    }


    int sendMotorsOrderCallback(const interfaces::msg::MotorsOrder & motorsOrder) {

      frame.can_id = ID_MOTORS_CMD;

      frame.data[0] = motorsOrder.left_rear_speed;
      frame.data[1] = motorsOrder.right_rear_speed;
      frame.data[2] = motorsOrder.steering_speed;

      return canSend(frame);
    }

    int sendCalibrationRequestCallback(const interfaces::msg::SteeringCalibration & calibrationMsg) {

      if (calibrationMsg.request == true){
        frame.can_id = ID_CALIBRATION_MODE;
        frame.data[0] = CALIBRATION_REQUEST;

        return canSend(frame);
      }

      return 0;
      
    }

    int canSend(struct can_frame frame){

      if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Send error");
        return 1;
      }

      RCLCPP_DEBUG(this->get_logger(), "Sending success, can ID 0x%03X", frame.can_id);
      return 0;

    }

    int s; 
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    rclcpp::Subscription<interfaces::msg::MotorsOrder>::SharedPtr subscription_motors_order_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<can_tx>();
  rclcpp::spin(node);

  node->closeCommunication();
  rclcpp::shutdown();
  return 0;
}