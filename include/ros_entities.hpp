#pragma once

#include <memory>

#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
// #include <nav_msgs/msg/odometry.h> for publishing later

#include "pins.hpp"



// micro_ros definitions
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This microcontroller is only avaliable for Arduino framework with serial transport.
#endif


#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}



/////// Callback headers (defined in main.cpp)
void cmd_vel_callback(const void * msgin);
void diagnostics_callback(rcl_timer_t * timer, int64_t last_call_time);



/////// ROS 2 Entities
struct RosEntities {
    ~RosEntities();

    rcl_subscription_t cmd_vel_subcription;
    geometry_msgs__msg__Twist cmd_vel_msg;

    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t microros_node;

    rcl_publisher_t diagnostics_publisher;
    rcl_timer_t diagnostics_timer;
    std_msgs__msg__String diagnostics_msg;


    bool support_initialized = false;
    bool executor_initialized = false;
    bool node_initialized = false;
    bool subscription_initialized = false;
    bool publisher_initialized = false;
    bool diagnostics_msg_initialized = false;
    bool timer_initialized = false;
};



/**
 * Sets up the microros structures, nodes, topics, and other things living in RosEntities.
 * On success returns a RosEntities pointer with all these things
 * On failure enter a busy loop forever, never leaving this function.
 */
std::unique_ptr<RosEntities>  microros_setup() {
    auto  ros_entities =  std::make_unique<RosEntities>();

    ros_entities->allocator = rcl_get_default_allocator();


    // Create init options / support
    if (rclc_support_init(
            &ros_entities->support,
            0,
            NULL,
            &ros_entities->allocator
        ) != RCL_RET_OK) {
        ros_entities.reset();
    }
    ros_entities->support_initialized = true;


    // Initialize executor
    ros_entities->executor = rclc_executor_get_zero_initialized_executor();

    if (rclc_executor_init(
            &ros_entities->executor,
            &ros_entities->support.context,
            2,  // subscriptions + timers
            &ros_entities->allocator
        ) != RCL_RET_OK) {
        ros_entities.reset();
    }
    ros_entities->executor_initialized = true;


    // Create node
    if (rclc_node_init_default(
            &ros_entities->microros_node,
            "micro_ros_platformio_node",
            "",
            &ros_entities->support
        ) != RCL_RET_OK) {
        ros_entities.reset();
    }
    ros_entities->node_initialized = true;


    // Create subscriber
    if (rclc_subscription_init_default(
            &ros_entities->cmd_vel_subcription,
            &ros_entities->microros_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel"
        ) != RCL_RET_OK) {
        ros_entities.reset();
    }
    ros_entities->subscription_initialized = true;


    // Add subscription to executor
    if (rclc_executor_add_subscription(
            &ros_entities->executor,
            &ros_entities->cmd_vel_subcription,
            &ros_entities->cmd_vel_msg,
            &cmd_vel_callback,
            ON_NEW_DATA
        ) != RCL_RET_OK) {
        ros_entities.reset();
    }


    // Initialize diagnostics message
    if (!std_msgs__msg__String__init(&ros_entities->diagnostics_msg)) {
        ros_entities.reset();
    }
    ros_entities->diagnostics_msg_initialized = true;


    // Create diagnostics publisher
    if (rclc_publisher_init_default(
            &ros_entities->diagnostics_publisher,
            &ros_entities->microros_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/diagnostics"
        ) != RCL_RET_OK) {
        ros_entities.reset();
    }
    ros_entities->publisher_initialized = true;


    // Create diagnostics timer
    if (rclc_timer_init_default(
            &ros_entities->diagnostics_timer,
            &ros_entities->support,
            RCL_MS_TO_NS(1000),
            &diagnostics_callback
        ) != RCL_RET_OK) {
        ros_entities.reset();
    }
    ros_entities->timer_initialized = true;


    // Add timer to executor
    if (rclc_executor_add_timer(
            &ros_entities->executor,
            &ros_entities->diagnostics_timer
        ) != RCL_RET_OK) {
        ros_entities.reset();
    }

    return ros_entities;
}


/**
 * Destroys everything in a given RosEntities object correctly
 */
RosEntities::~RosEntities() {
    if (this == nullptr) return;

    // Remove executor and its handles
    if (this->executor_initialized)
        RCSOFTCHECK(rclc_executor_fini(&this->executor));

    // Destroy ROS entities
    if (this->timer_initialized)
        RCSOFTCHECK(rcl_timer_fini(&this->diagnostics_timer));
    
    if (this->publisher_initialized) {
        RCSOFTCHECK(rcl_publisher_fini(
            &this->diagnostics_publisher,
            &this->microros_node
        ));
    }

    if (this->subscription_initialized) {
        RCSOFTCHECK(rcl_subscription_fini(
            &this->cmd_vel_subcription,
            &this->microros_node
        ));
    }

    // Destroy node and support
    if (this->node_initialized)
        RCSOFTCHECK(rcl_node_fini(&this->microros_node));
    if (this->support_initialized)
        RCSOFTCHECK(rclc_support_fini(&this->support));

    // Free dynamically allocated message data
    if (this->diagnostics_msg_initialized)
        std_msgs__msg__String__fini(&this->diagnostics_msg);
}


////// Heartbeat check
unsigned long lastAgentCheck = 0;
bool agentConnected = false;

/**
 * Check agent connection and spin nodes if we are connected
 * 
*/
void checkAgentAndSpin(std::unique_ptr<RosEntities> & ros_entities) {
    unsigned long now = millis();

    if (now - lastAgentCheck >= 1000) {
        lastAgentCheck = now;

        agentConnected = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);

        // destroy and rebuild on connection failure
        if (!agentConnected) {
            if (ros_entities != nullptr) {
                ros_entities.reset(); // sets to nullptr for next loop
            }
        }
        else if (ros_entities == nullptr) {
            ros_entities = microros_setup();
        }

        // Spin ros nodes
        if (ros_entities != nullptr) {
            rclc_executor_spin_some(
                &(ros_entities->executor),
                RCL_MS_TO_NS(1)
            );
        }

    }
}