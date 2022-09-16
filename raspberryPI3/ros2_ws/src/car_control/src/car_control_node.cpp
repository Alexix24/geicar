#include <stdint.h>
#include <string.h>  
#include <map>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/steering_calibration.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/empty.hpp"

#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/propulsionCmd.h"

using namespace std;
using placeholders::_1;

#define DEADZONE_LT_RT 15.0  // %
#define DEADZONE_LS_X 20.0  // %
#define STOP 50



class car_control : public rclcpp::Node {

public:
    car_control()
    : Node("car_control_node")
    {
        startCar=false;
        requestedSpeed = STOP;
        requestedAngle = STOP;
        
        axisMap.insert({"LS_X",0});    //Left Stick axis X  (full left) 1.0 > -1.0 (full right)
        axisMap.insert({"LS_Y",1});    //Left Stick axis Y  (full high) 1.0 > -1.0 (full bottom)
        axisMap.insert({"LT",2});      //LT  (high) 1.0 > -1.0 (bottom)
        axisMap.insert({"RS_X",3});    //Right Stick axis X  (full left) 1.0 > -1.0 (full right)
        axisMap.insert({"RS_Y",4});    //Right Stick axis Y  (full high) 1.0 > -1.0 (full bottom)
        axisMap.insert({"RT",5});      //RT  (high) 1.0 > -1.0 (bottom)
        axisMap.insert({"DPAD_X",6});  //(left) 1.0 > -1.0 (right)
        axisMap.insert({"DPAD_Y",7});  //(hight) 1.0 > -1.0 (bottom)

        buttonsMap.insert({"A",0});
        buttonsMap.insert({"B",1});
        buttonsMap.insert({"Y",2});
        buttonsMap.insert({"X",3});
        buttonsMap.insert({"LB",4});
        buttonsMap.insert({"RB",5});
        buttonsMap.insert({"START",6});
        buttonsMap.insert({"SELECT",7});
        buttonsMap.insert({"XBOX",8});
        buttonsMap.insert({"LS",9});   //Left Stick Button
        buttonsMap.insert({"RS",10});  //Right Stick Button


        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("send_motors_order", 10);

        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);

        subscription_steringCalibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
        "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));

        subscription_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&car_control::joyCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        timer_ = this->create_wall_timer(5ms, std::bind(&car_control::updateCmd, this));

        server_calibration_ = this->create_service<std_srvs::srv::Empty>(
                            "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));

        
        RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

    
private:

    //Update requestedSpeed and requestedAngle from the joystick
    void joyCallback(const sensor_msgs::msg::Joy & joy) {

        buttonStart = joy.buttons[buttonsMap.find("START")->second];
        buttonB = joy.buttons[buttonsMap.find("B")->second];    

        axisRT = joy.axes[axisMap.find("RT")->second];      //Motors (go forward)
        axisLT = joy.axes[axisMap.find("LT")->second];      //Motors (go backward)
        axisLS_X = joy.axes[axisMap.find("LS_X")->second];     //Steering


        //Normalise values beetween 0 and 100
        axisLS_X = 50.0 * (1.0-axisLS_X);   
        axisRT = 75.0 - 25.0*axisRT;
        axisLT = 25.0+25.0*axisLT;


        // ------ Start and Stop ------
        if (!startCar && buttonStart && !buttonB){       // Start button -> Start the car    
            carMode = 0;
            startCar = true;
            RCLCPP_INFO(this->get_logger(), "START");
        }

        if (buttonB && startCar){       // B button -> Stop the car
            startCar = false;
            requestedSpeed = STOP;
            RCLCPP_INFO(this->get_logger(), "STOP");
        }


        if (startCar){

            // ------ Propulsion ------
            if (axisLT < (50.0-DEADZONE_LT_RT) && axisRT > (50.0+DEADZONE_LT_RT)){  //Incompatible orders : Stop the car
                requestedSpeed = STOP;
                RCLCPP_WARN(this->get_logger(), "Incompatible orders");

            } else if (axisLT > (50.0-DEADZONE_LT_RT) && axisRT < (50.0+DEADZONE_LT_RT)){ 
                requestedSpeed = STOP;
            
            }else if (axisLT < (50.0-DEADZONE_LT_RT)){ //Move backward
                requestedSpeed = static_cast<int>(axisLT);

            } else if (axisRT > (50.0+DEADZONE_LT_RT)){   //Move forward
                requestedSpeed = static_cast<int>(axisRT);
            }


            // ------ Steering ------
            if (axisLS_X > (50.0 - DEADZONE_LS_X) && axisLS_X < 55.0 ){     //asymmetric deadzone (hardware : joystick LS)
                requestedAngle = 50;
            } else {
                requestedAngle = static_cast<int>(axisLS_X);    //axisLS_X : -1 .. 1  ;  steering_angle : 0 .. 100
            }

        }
    }


    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
        currentAngle = motorsFeedback.steering_angle;

        //AL : eventuellement rajoute leftRearCurrentSpeed et rightRearCurrentSpeed, qui seront passe en arg de propulsionCmd()
    }


    //Update the command
    void updateCmd(){
        
        if (carMode!=2){
            auto motorsOrder = interfaces::msg::MotorsOrder();  //By default, motorsOrder stop the car (see interfaces/msg/MotorsOrder.msg)

            propulsionCmd(requestedSpeed,currentSpeed,leftRearSpeedCmd, rightRearSpeedCmd);

            steeringCmd(requestedAngle,currentAngle, steeringSpeedCmd);
            
            motorsOrder.left_rear_speed = leftRearSpeedCmd;
            motorsOrder.right_rear_speed = rightRearSpeedCmd;
            motorsOrder.steering_speed = steeringSpeedCmd;

            publisher_can_->publish(motorsOrder);   //Send order

        }
        
    }

    //Function called by "steering_calibration" service
    void steeringCalibration(std_srvs::srv::Empty::Request::SharedPtr req,
                  std_srvs::srv::Empty::Response::SharedPtr res)
    {

        carMode = 2;    //Switch to calibration mode

        auto calibrationMsg = interfaces::msg::SteeringCalibration();
        calibrationMsg.request = true;

        RCLCPP_INFO(this->get_logger(), "Sending calibration request .....");
        publisher_steeringCalibration_->publish(calibrationMsg);

    }
    
    void steeringCalibrationCallback (const interfaces::msg::SteeringCalibration & calibrationMsg){

        if (calibrationMsg.in_progress == true && calibrationMsg.user_need == false){
        RCLCPP_INFO(this->get_logger(), "Steering Calibration in progress, please wait ....");

        } else if (calibrationMsg.in_progress == true && calibrationMsg.user_need == true){
            RCLCPP_INFO(this->get_logger(), "Please use the buttons (L/R) to center the steering wheels.\nThen, press the blue button on the NucleoF103 to continue");
        } else if (calibrationMsg.status == 1){
            RCLCPP_INFO(this->get_logger(), "Steering calibration [SUCCESS]");
        } else if (calibrationMsg.status == -1){
            RCLCPP_ERROR(this->get_logger(), "Steering calibration [FAILED]");
        }
        
    }
    
    bool startCar;

    //Manual Mode variables (with joystick control)
    map<string,int> axisMap;
    map<string,int> buttonsMap;
    bool buttonB, buttonStart;
    int carMode;    //0 : Manual    1 : Auto    2 : Calibration
    float axisRT, axisLT, axisLS_X;


    //Control variables
    uint8_t requestedAngle, currentAngle, requestedSpeed, currentSpeed; //Current speed sera renvoyé par le node IMU
    uint8_t leftRearSpeedCmd;
    uint8_t rightRearSpeedCmd;
    uint8_t steeringSpeedCmd;


    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joy_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steringCalibration_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_calibration_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_control>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}