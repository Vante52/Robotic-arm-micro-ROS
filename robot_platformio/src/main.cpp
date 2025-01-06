#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int64_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/int64.h>

#include "constants.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid.h"
#include "hw.h"


diffrobot::MotorDriver left_motor(diffrobot::Hw::kLeftMotorEnableGpioPin,
                                  diffrobot::Hw::kLeftMotorForwardGpioPin,
                                  diffrobot::Hw::kLeftMotorBackwardGpioPin);

diffrobot::MotorDriver right_motor(diffrobot::Hw::kRightMotorEnableGpioPin,
                                   diffrobot::Hw::kRightMotorForwardGpioPin,
                                   diffrobot::Hw::kRightMotorBackwardGpioPin);

diffrobot::EncoderDriver left_encoder(diffrobot::Hw::kLeftEncoderChannelAGpioPin,
                                      diffrobot::Hw::kLeftEncoderChannelBGpioPin);

diffrobot::EncoderDriver right_encoder(diffrobot::Hw::kRightEncoderChannelAGpioPin,
                                       diffrobot::Hw::kRightEncoderChannelBGpioPin);                          

diffrobot::Pid left_pid_controller(diffrobot::Constants::kPidKp,
                                   diffrobot::Constants::kPidKd,
                                   diffrobot::Constants::kPidKi,
                                   diffrobot::Constants::kPidKo,
                                   - diffrobot::Constants::kPwmMax,
                                   diffrobot::Constants::kPwmMax);

diffrobot::Pid right_pid_controller(diffrobot::Constants::kPidKp,
                                   diffrobot::Constants::kPidKd,
                                   diffrobot::Constants::kPidKi,
                                   diffrobot::Constants::kPidKo,
                                   - diffrobot::Constants::kPwmMax,
                                   diffrobot::Constants::kPwmMax);

void IRAM_ATTR read_left_enc();
void IRAM_ATTR read_right_enc();
void adjust_motors_speed();
void set_motor_speed(int left_speed, int right_speed);

rcl_publisher_t right_encoder_publisher;
rcl_publisher_t left_encoder_publisher;
rcl_subscription_t motor_speed_subs;

std_msgs__msg__Int64 left_encoder_value;
std_msgs__msg__Int64 right_encoder_value;

std_msgs__msg__Int64MultiArray incoming_msg_speed;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    adjust_motors_speed();
    left_encoder_value.data = left_encoder.read();
    right_encoder_value.data = right_encoder.read();

    RCSOFTCHECK(rcl_publish(&left_encoder_publisher, (const void*)&left_encoder_value, NULL));
    RCSOFTCHECK(rcl_publish(&right_encoder_publisher, (const void*)&right_encoder_value, NULL));
  }
}

void cmd_motor_callback(const void *msgin) {
  const std_msgs__msg__Int64MultiArray * msg = (const std_msgs__msg__Int64MultiArray *)msgin;
  set_motor_speed(msg->data.data[0], msg->data.data[1]);
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  // set_microros_serial_transports(Serial);
  IPAddress agent_ip(127, 0, 0, 0); // PC Local Ip
  size_t port = 8888;
  char ssid[] = "SSID";
  char psk[] = "PASSWORD";
  set_microros_wifi_transports(ssid, psk, agent_ip, port);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &right_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "diffrobot_right_position"));
  
  RCCHECK(rclc_publisher_init_default(
    &left_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "diffrobot_left_position"));
  
  RCCHECK(rclc_subscription_init_default(
    &motor_speed_subs,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
    "diffrobot_motor"));
  
  // create timer,
  const unsigned int timer_timeout = 1000 / diffrobot::Constants::kPidRate;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  incoming_msg_speed.data.capacity = 2;
  incoming_msg_speed.data.size = 0;
  incoming_msg_speed.data.data = (int64_t*) malloc(incoming_msg_speed.data.capacity * sizeof(int64_t));

  incoming_msg_speed.layout.dim.capacity = 2;
  incoming_msg_speed.layout.dim.size = 0;
  incoming_msg_speed.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(incoming_msg_speed.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for(size_t i = 0; i < incoming_msg_speed.layout.dim.capacity; i++){
      incoming_msg_speed.layout.dim.data[i].label.capacity = 2;
      incoming_msg_speed.layout.dim.data[i].label.size = 0;
      incoming_msg_speed.layout.dim.data[i].label.data = (char*) malloc(incoming_msg_speed.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_speed_subs, &incoming_msg_speed, &cmd_motor_callback, ON_NEW_DATA));

  left_encoder_value.data = 0.0;
  right_encoder_value.data = 0.0;

  left_motor.begin();
  right_motor.begin();

  left_encoder.begin();
  right_encoder.begin();

  attachInterrupt(diffrobot::Hw::kLeftEncoderChannelAGpioPin, read_left_enc, CHANGE);
  attachInterrupt(diffrobot::Hw::kLeftEncoderChannelBGpioPin, read_left_enc, CHANGE);
  attachInterrupt(diffrobot::Hw::kRightEncoderChannelAGpioPin, read_right_enc, CHANGE);
  attachInterrupt(diffrobot::Hw::kRightEncoderChannelBGpioPin, read_right_enc, CHANGE);

  left_pid_controller.reset(left_encoder.read());
  right_pid_controller.reset(right_encoder.read());
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}


void IRAM_ATTR read_left_enc() {
  left_encoder.readEncoder();
}

void IRAM_ATTR read_right_enc() {
  right_encoder.readEncoder();
}

void stop_motors() {
  left_motor.set_speed(0);
  right_motor.set_speed(0);

  left_pid_controller.disable();
  right_pid_controller.disable();
}

void set_motor_speed(int left_speed, int right_speed) {
  const int left_motor_speed = left_speed;
  const int right_motor_speed = right_speed;

  if (left_motor_speed == 0 && right_motor_speed == 0) {
    left_motor.set_speed(0);
    right_motor.set_speed(0);
    left_pid_controller.reset(left_encoder.read());
    right_pid_controller.reset(right_encoder.read());
    left_pid_controller.disable();
    right_pid_controller.disable();
  } else {
    left_pid_controller.enable();
    right_pid_controller.enable();
  }

  left_pid_controller.set_setpoint(left_motor_speed / diffrobot::Constants::kPidRate);
  right_pid_controller.set_setpoint(right_motor_speed / diffrobot::Constants::kPidRate);
}


void adjust_motors_speed() {
  int left_motor_speed = 0;
  int right_motor_speed = 0;

  left_pid_controller.compute(left_encoder.read(), left_motor_speed);
  right_pid_controller.compute(right_encoder.read(), right_motor_speed);
  if (left_pid_controller.enabled()) {
    left_motor.set_speed(left_motor_speed);
  }

  if (right_pid_controller.enabled()) {
    right_motor.set_speed(right_motor_speed);
  }
}