#pragma once
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    uint32_t stop_pulse_width_us;    // 停止脉冲宽度（微秒）
    uint32_t max_fwd_pulse_width_us; // 最大正向脉冲宽度（微秒）
    uint32_t max_rev_pulse_width_us; // 最大反向脉冲宽度（微秒）
} servo_pwm_range_t;

class Servo360 {
public:
    Servo360(gpio_num_t gpio, mcpwm_timer_handle_t timer, const servo_pwm_range_t* pwm_range = nullptr, bool reverse = false);
    ~Servo360();
    esp_err_t setup_pwm();
    void set_speed(int speed, int duration_ms = 0);
    void stop();
    void run_for(int speed, int duration_ms);
    void quick_action(int speed, int duration_ms); // 快速响应动作
    void back_and_forth(int speed, int duration_ms, int count = 1); // 来回动作

    // 复合动作
    void raise_arm(int speed, int duration_ms);
    void wave(int speed, int duration_ms, int count);
    void salute(int speed, int duration_ms);

    int get_current_speed() const;
    gpio_num_t get_gpio() const;
    // 直接设置 PWM 脉冲宽度（用于校准）
    void set_raw_pulse_width(uint32_t pulse_width_us);


private:
    gpio_num_t gpio_;
    mcpwm_timer_handle_t timer_;
    mcpwm_oper_handle_t oper_;
    mcpwm_cmpr_handle_t cmpr_;
    mcpwm_gen_handle_t gen_;
    int current_speed_;
    servo_pwm_range_t pwm_range_;
    bool initialized_;
    bool reverse_;
    
    void set_speed_internal(int speed);
};