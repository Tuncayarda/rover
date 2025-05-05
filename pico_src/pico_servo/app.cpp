
 #include <stdio.h>
 #include <time.h>
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
   #include <rover_interfaces/msg/servo_command.h>
   #include "pico_uart_transports.h"
 }
 
 // PWM ayarları
 #define PWM_FREQ_HZ 333.0f
 #define PWM_WRAP    2000
 
 static const uint PWM_PINS[6] = {16, 17, 18, 19, 20, 21};
 static uint8_t pwm_slices[6];
 static uint8_t pwm_chans[6];
 
 // LED pini, ping durumu göstermek için
 const uint LED_PIN = 25;
 
 // Durum makinesi için enum
 enum states {
   WAITING_AGENT,
   AGENT_AVAILABLE,
   AGENT_CONNECTED,
   AGENT_DISCONNECTED
 } state;
 
 // micro-ROS varlıkları
 rcl_subscription_t subscriber;
 rover_interfaces__msg__ServoCommand servo_msg;
 rclc_executor_t executor;
 rcl_node_t node;
 rclc_support_t support;
 rcl_allocator_t allocator;
 
 // Gelen ServoCommand mesajı callback’i
 void servo_command_callback(const void * msg_in)
 {
     auto msg = static_cast<const rover_interfaces__msg__ServoCommand *>(msg_in);
     float period_us = 1e6f / PWM_FREQ_HZ;
     for (int i = 0; i < 6; i++) {
         int8_t angle = msg->angles[i];
         float pulse_us = (angle / 180.0f) * 1000.0f + 1000.0f;
         uint32_t level = uint32_t((pulse_us * (PWM_WRAP + 1)) / period_us);
         pwm_set_chan_level(pwm_slices[i], pwm_chans[i], level);
     }
 }
 
 // Agent’e ping atıp LED durumunu güncelleyen fonksiyon
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
 
 // PWM kanallarını ve dilimlerini başlatır
 void init_pwm()
 {
     for (int i = 0; i < 6; i++) {
         gpio_init(PWM_PINS[i]);
         gpio_set_function(PWM_PINS[i], GPIO_FUNC_PWM);
         pwm_slices[i] = pwm_gpio_to_slice_num(PWM_PINS[i]);
         pwm_chans[i]  = pwm_gpio_to_channel(PWM_PINS[i]);
         pwm_set_wrap(pwm_slices[i], PWM_WRAP);
         float clkdiv = float(clock_get_hz(clk_sys)) / (PWM_FREQ_HZ * (PWM_WRAP + 1));
         pwm_set_clkdiv(pwm_slices[i], clkdiv);
         pwm_set_chan_level(pwm_slices[i], pwm_chans[i], 0);
         pwm_set_enabled(pwm_slices[i], true);
     }
 }
 
 // micro-ROS node, subscription ve executor’ı oluşturur
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
 
 // micro-ROS varlıklarını sonlandırır
 void destroyEntities()
 {
     rcl_subscription_fini(&subscriber, &node);
     rclc_executor_fini(&executor);
     rcl_node_fini(&node);
     rclc_support_fini(&support);
 }
 
 int main()
 {
     stdio_init_all();
 
     // LED pini
     gpio_init(LED_PIN);
     gpio_set_dir(LED_PIN, GPIO_OUT);
     gpio_put(LED_PIN, 0);
 
     // PWM başlat
     init_pwm();
 
     // Seri transportu yapılandır
     rmw_uros_set_custom_transport(
         true,
         NULL,
         pico_serial_transport_open,
         pico_serial_transport_close,
         pico_serial_transport_write,
         pico_serial_transport_read
     );
 
     // Durum makinesini başlat
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
                     // Gelen veri var mı diye kısa süreli dön
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