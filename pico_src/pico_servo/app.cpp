#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "servo.h"
 
extern "C" {
    #include <rcl/rcl.h>
    #include <rcl/error_handling.h>
    #include <rclc/rclc.h>
    #include <rclc/executor.h>
    #include <rmw_microros/rmw_microros.h>
    #include <rover_interfaces/msg/servo_command.h>
    #include "pico_uart_transports.h"
}
 
static const uint PWM_PINS[6] = {16, 17, 18, 19, 20, 21};
static Servo servos[6];
static const int SERVO_CENTER[6] = {115, 115, 120, 85, 85, 90};
 
const uint LED_PIN = 25;
 
enum states {
   WAITING_AGENT,
   AGENT_AVAILABLE,
   AGENT_CONNECTED,
   AGENT_DISCONNECTED
} state;
 
rcl_subscription_t subscriber;
rover_interfaces__msg__ServoCommand servo_msg;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Callback for processing incoming ServoCommand messages;
// computes target positions using center offsets and drives servos.
void servo_command_callback(const void * msg_in)
{
    auto msg = static_cast<const rover_interfaces__msg__ServoCommand *>(msg_in);
    for (int i = 0; i < 6; i++) {
        int cmd = msg->angles[i];
        int center = SERVO_CENTER[i];
        int target = cmd + center;
        printf("Servo %d: cmd=%d + center=%d -> target=%d\n", i, cmd, center, target);
        servos[i].goDegree(target);
    }
}

// Ping the micro-ROS agent and update the LED indicator based on the response.
bool pingAgent()
{
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
        gpio_put(LED_PIN, 0);
        return false;
    }
    gpio_put(LED_PIN, 1);
    return true;
}

// Initialize PWM channels for each servo based on configured GPIO pins.
void init_pwm()
{
    for (int i = 0; i < 6; i++) {
        servos[i].setGP(PWM_PINS[i]);
    }
}

// Create micro-ROS node, subscription, and executor for servo commands.
void createEntities()
{
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "servo_control_node", "", &support);

    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rover_interfaces, msg, ServoCommand),
        "servo_command");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &servo_msg,
        &servo_command_callback,
        ON_NEW_DATA);
}

// Destroy micro-ROS entities: subscription, executor, node, and support.
void destroyEntities()
{
    rcl_subscription_fini(&subscriber, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// Main loop: initialize hardware, configure serial transport,
// and run state machine to manage micro-ROS agent connection.
int main()
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    init_pwm();

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    state = WAITING_AGENT;
    while (true) {
        switch (state) {
            case WAITING_AGENT:
                state = pingAgent() ? AGENT_AVAILABLE : WAITING_AGENT;
                break;

            case AGENT_AVAILABLE:
                createEntities();
                state = AGENT_CONNECTED;
                break;

            case AGENT_CONNECTED:
                if (!pingAgent()) {
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