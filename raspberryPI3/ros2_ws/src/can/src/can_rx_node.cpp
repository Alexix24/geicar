#include <stdint.h>
#include <math.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/general_data.hpp"
#include "interfaces/msg/steering_calibration.hpp"

#include "../include/can/can.h"



class can_rx : public rclcpp::Node {

public:
    can_rx()
    : Node("can_rx_node")
    {
        // publisher_us_ = this->create_publisher<interfaces::msg::TrameCan>("us_data", 10);
        // publisher_imu_ = this->create_publisher<interfaces::msg::TrameCan>("imu_data", 10);
        // publisher_gps_ = this->create_publisher<interfaces::msg::TrameCan>("gps_data", 10);
        publisher_motorsFeedback_ = this->create_publisher<interfaces::msg::MotorsFeedback>("motors_feedback", 10);
        publisher_generalData_ = this->create_publisher<interfaces::msg::GeneralData>("general_data", 10);
        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);


        if (initCommunication()==0){
            readData(); 
            closeCommunication();
        }
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

        RCLCPP_INFO(this->get_logger(), "CAN communication initialized");
        return 0;
    }

    int readData(){

        RCLCPP_INFO(this->get_logger(), "Reading CAN bus ...");

        while (rclcpp::ok()){
            
            nbytes = read(s, &frame, sizeof(struct can_frame));

            if (nbytes < 0) {
                RCLCPP_ERROR(this->get_logger(), "Read error");
                return 1;
            }

            RCLCPP_DEBUG(this->get_logger(), "Read value (ID) : 0x%03X", frame.can_id);

            for (i = 0; i < frame.can_dlc; i++)
                RCLCPP_DEBUG(this->get_logger(), "Read value (Data) : %02X", frame.data[i]);
                

            // auto message = interfaces::msg::TrameCan();
            // message.id = frame.can_id;
            // message.dlc = frame.can_dlc;

            // for (int i=0 ; i<frame.can_dlc ; i++){
            //     message.data.push_back(frame.data[i]);
            // }
            
            
            if (frame.can_id==ID_SSB_DATAS){
                auto motorsFeedbackMsg = interfaces::msg::MotorsFeedback();
                auto generalDataMsg = interfaces::msg::GeneralData();

                //Battery level
                float batMes = (frame.data[0] << 8) + frame.data[1];
                batMes = batMes * (11.65 / 2794.0); //Convert to Volts

                generalDataMsg.battery_level = batMes;

                //Motors
                float leftSpeedMes = (frame.data[2] << 8) + frame.data[3];
                float rightSpeedMes = (frame.data[4] << 8) + frame.data[5];

                motorsFeedbackMsg.left_rear_speed = 0.01 * leftSpeedMes ; 
                motorsFeedbackMsg.right_rear_speed = 0.01 * rightSpeedMes ; 
                motorsFeedbackMsg.steering_angle = frame.data[6];


                //Publication on topics
                RCLCPP_DEBUG(this->get_logger(), "Publishing to motors_feedback and general_data topics");
                publisher_motorsFeedback_->publish(motorsFeedbackMsg);
                publisher_generalData_->publish(generalDataMsg);

            } else if (frame.can_id == ID_CALIBRATION_MODE){
                auto calibrationMsg = interfaces::msg::SteeringCalibration();
                
                if (frame.data[0]==CALIBRATION_IN_PROGRESS){
                    calibrationMsg.in_progress = true;

                }else if (frame.data[0]==CALIBRATION_SUCCESS){
                    calibrationMsg.status = 1;
                }else if (frame.data[0]==CALIBRATION_FAIL){
                    calibrationMsg.status = -1;
                }

                if (frame.data[1]==CALIBRATION_USER_NEED){
                    calibrationMsg.user_need = true;
                }else{
                    calibrationMsg.user_need = false;
                }

                publisher_steeringCalibration_->publish(calibrationMsg);

            }
            // else if (frame.can_id==ID_US){
            //     RCLCPP_DEBUG(this->get_logger(), "Publishing to us_data Topic");
            //     publisher_us_->publish(message);

            // }else if (frame.can_id==ID_IMU){
            //     RCLCPP_DEBUG(this->get_logger(), "Publishing to imu_data Topic");
            //     publisher_imu_->publish(message);

            // }else if (frame.can_id==ID_GPS){
            //     RCLCPP_DEBUG(this->get_logger(), "Publishing to gps_data Topic");
            //     publisher_gps_->publish(message);
            // }

        }

        return 0;
    }

    int closeCommunication(){

        if (close(s) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Close communication failed - Socket close error");
            return 1;
        }
        RCLCPP_INFO(this->get_logger(), "Socket closed");
        return 0;
    }

    int s, i; 
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    // rclcpp::Publisher<interfaces::msg::TrameCan>::SharedPtr publisher_us_;
    // rclcpp::Publisher<interfaces::msg::TrameCan>::SharedPtr publisher_imu_;
    // rclcpp::Publisher<interfaces::msg::TrameCan>::SharedPtr publisher_gps_;

    rclcpp::Publisher<interfaces::msg::MotorsFeedback>::SharedPtr publisher_motorsFeedback_;
    rclcpp::Publisher<interfaces::msg::GeneralData>::SharedPtr publisher_generalData_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;
    
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<can_rx>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}