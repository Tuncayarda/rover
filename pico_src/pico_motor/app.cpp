#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/time.h"

extern "C" {
    #include <rcl/rcl.h>
    #include <rcl/error_handling.h>
    #include <rclc/rclc.h>
    #include <rclc/executor.h>
    #include <rmw_microros/rmw_microros.h>
    #include <rover_interfaces/msg/motor_command.h>
    #include "pico_uart_transports.h"
}

// PWM settings
#define PWM_FREQ_HZ 1000.0f
#define PWM_WRAP    1000

// LED pin for agent status indicator
const uint LED_PIN = 25;

// Customize motor pin pairs: {forward_pin, reverse_pin}
static uint8_t MOTOR_PIN_PAIRS[6][2] = {
    {3, 2},    // Motor 1: forward on GP2, reverse on GP3
    {5, 4},    // Motor 2: forward on GP4, reverse on GP5
    {6, 7},    // Motor 3: forward on GP6, reverse on GP7
    {9, 8},    // Motor 4: forward on GP8, reverse on GP9
    {10, 11},  // Motor 5: forward on GP10, reverse on GP11
    {13, 12}   // Motor 6: forward on GP12, reverse on GP13
};

// Motor structure holding PWM channels
typedef struct {
    uint8_t pin_fwd;
    uint8_t pin_rev;
    uint  slice_fwd;
    uint  chan_fwd;
    uint  slice_rev;
    uint  chan_rev;
} Motor;

static Motor motors[6];

// micro-ROS entities
static rcl_subscription_t subscriber;
static rover_interfaces__msg__MotorCommand motor_msg;
static rclc_executor_t executor;
static rcl_node_t node;
static rclc_support_t support;
static rcl_allocator_t allocator;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

// Ping the micro-ROS agent and update LED status
bool pingAgent() {
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    gpio_put(LED_PIN, ret == RCL_RET_OK);
    return ret == RCL_RET_OK;
}

// Initialize PWM channels for motors based on pin pairs.
void init_pwm() {
    for(int i = 0; i < 6; i++) {
        motors[i].pin_fwd = MOTOR_PIN_PAIRS[i][0];
        motors[i].pin_rev = MOTOR_PIN_PAIRS[i][1];
        gpio_set_function(motors[i].pin_fwd, GPIO_FUNC_PWM);
        gpio_set_function(motors[i].pin_rev, GPIO_FUNC_PWM);
        motors[i].slice_fwd = pwm_gpio_to_slice_num(motors[i].pin_fwd);
        motors[i].chan_fwd  = pwm_gpio_to_channel(motors[i].pin_fwd);
        motors[i].slice_rev = pwm_gpio_to_slice_num(motors[i].pin_rev);
        motors[i].chan_rev  = pwm_gpio_to_channel(motors[i].pin_rev);
        pwm_set_wrap(motors[i].slice_fwd, PWM_WRAP);
        pwm_set_wrap(motors[i].slice_rev, PWM_WRAP);
        pwm_set_enabled(motors[i].slice_fwd, true);
        pwm_set_enabled(motors[i].slice_rev, true);
    }
}

void set_motor_pwm(uint motor_idx, int8_t dir, int16_t speed) {
    if (motor_idx >= 6) return;
    uint duty = (speed * PWM_WRAP) / 100;

    uint slice_fwd = motors[motor_idx].slice_fwd;
    uint chan_fwd  = motors[motor_idx].chan_fwd;
    uint slice_rev = motors[motor_idx].slice_rev;
    uint chan_rev  = motors[motor_idx].chan_rev;

    if (dir > 0) {
        pwm_set_chan_level(slice_fwd, chan_fwd, duty);
        pwm_set_chan_level(slice_rev, chan_rev, 0);
    } else if (dir < 0) {
        pwm_set_chan_level(slice_fwd, chan_fwd, 0);
        pwm_set_chan_level(slice_rev, chan_rev, duty);
    } else {
        pwm_set_chan_level(slice_fwd, chan_fwd, 0);
        pwm_set_chan_level(slice_rev, chan_rev, 0);
    }
}

// Callback for processing incoming MotorCommand messages; sets PWM outputs based on direction and speed arrays.
void motor_command_callback(const void * msg_in) {
    const auto *msg = static_cast<const rover_interfaces__msg__MotorCommand *>(msg_in);
    for(int i = 0; i < 6; i++) {
        set_motor_pwm(i, msg->direction[i], msg->speed[i]);
    }
}

// Create micro-ROS node, subscription, and executor for motor commands.
void createEntities() {
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "motor_control_node", "", &support);
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rover_interfaces, msg, MotorCommand),
        "motor_command");
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1));
    rclc_executor_add_subscription(&executor, &subscriber, &motor_msg, &motor_command_callback, ON_NEW_DATA);
}

// Destroy micro-ROS entities: subscription, executor, node, and support.
void destroyEntities() {
    rcl_subscription_fini(&subscriber, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// Main loop: initialize hardware, configure serial transport, and manage micro-ROS agent connection for motor control.
int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    init_pwm();
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);
    state = WAITING_AGENT;
    while(true) {
        switch(state) {
            case WAITING_AGENT:
                state = pingAgent() ? AGENT_AVAILABLE : WAITING_AGENT;
                break;
            case AGENT_AVAILABLE:
                createEntities();
                state = AGENT_CONNECTED;
                break;
            case AGENT_CONNECTED:
                if(!pingAgent()) {
                    state = AGENT_DISCONNECTED;
                } else {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                }
                break;
            case AGENT_DISCONNECTED:
                destroyEntities();
                state = WAITING_AGENT;
                break;
        }
    }
    return 0;
}
