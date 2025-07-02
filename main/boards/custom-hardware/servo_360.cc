#include "servo_360.h"
#include <esp_log.h>
#include <esp_log_level.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#define SERVO_PWM_FREQ 50  // 50Hz
#undef TAG
#define TAG "Servo360"

// 默认 SG90 360°连续旋转舵机 PWM 范围（微秒）
// 针对连续旋转舵机优化，避免震动
static const servo_pwm_range_t default_pwm_range = {
    .stop_pulse_width_us = 1500,    // 1.5ms - 停止
    .max_fwd_pulse_width_us = 2000, // 2.0ms - 最大正向
    .max_rev_pulse_width_us = 1000  // 1.0ms - 最大反向
};

Servo360::Servo360(gpio_num_t gpio, mcpwm_timer_handle_t timer, const servo_pwm_range_t* pwm_range, bool reverse)
    : gpio_(gpio), timer_(timer), current_speed_(0), initialized_(false), reverse_(reverse)
{
    ESP_LOGI(TAG, "Servo360 构造: GPIO=%d, Timer=%p, Reverse=%d", gpio_, timer_, reverse_);
    if (gpio_ == GPIO_NUM_NC || timer_ == nullptr) {
        ESP_LOGE(TAG, "无效参数: GPIO=%d, Timer=%p", gpio_, timer_);
        return;
    }
    if (pwm_range) {
        pwm_range_ = *pwm_range;
        ESP_LOGI(TAG, "使用自定义 PWM 范围: fwd=%lu, stop=%lu, rev=%lu", pwm_range_.max_fwd_pulse_width_us, pwm_range_.stop_pulse_width_us, pwm_range_.max_rev_pulse_width_us);
    } else {
        pwm_range_ = default_pwm_range;
        ESP_LOGI(TAG, "使用默认 PWM 范围: fwd=%lu, stop=%lu, rev=%lu", pwm_range_.max_fwd_pulse_width_us, pwm_range_.stop_pulse_width_us, pwm_range_.max_rev_pulse_width_us);
    }
    esp_err_t err = setup_pwm();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MCPWM 初始化失败，GPIO: %d, 错误: %d", gpio_, err);
    } else {
        ESP_LOGI(TAG, "MCPWM 初始化成功，GPIO: %d, Reverse: %d, oper_=%p, cmpr_=%p, gen_=%p", gpio_, reverse_, oper_, cmpr_, gen_);
        initialized_ = true;
    }
    stop();
}

Servo360::~Servo360() {
    ESP_LOGI(TAG, "Servo360 析构: GPIO=%d, oper_=%p, cmpr_=%p, gen_=%p, initialized_=%d", gpio_, oper_, cmpr_, gen_, initialized_);
    if (initialized_) {
        stop();
        if (gen_) mcpwm_del_generator(gen_);
        if (cmpr_) mcpwm_del_comparator(cmpr_);
        if (oper_) mcpwm_del_operator(oper_);
        ESP_LOGI(TAG, "MCPWM 资源已清理，GPIO: %d", gpio_);
    }
}

esp_err_t Servo360::setup_pwm() {
    esp_err_t err;
    ESP_LOGI(TAG, "[setup_pwm] GPIO=%d, group_id=0, timer_=%p", gpio_, timer_);
    initialized_ = false;
    mcpwm_operator_config_t oper_config = { .group_id = 0 };
    err = mcpwm_new_operator(&oper_config, &oper_);
    ESP_LOGI(TAG, "[setup_pwm] mcpwm_new_operator: oper_=%p, err=%d", oper_, err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建 MCPWM 操作器失败，GPIO: %d, 错误: %d", gpio_, err);
        return err;
    }
    err = mcpwm_operator_connect_timer(oper_, timer_);
    ESP_LOGI(TAG, "[setup_pwm] mcpwm_operator_connect_timer: oper_=%p, timer_=%p, err=%d", oper_, timer_, err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "连接定时器到操作器失败，GPIO: %d, 错误: %d", gpio_, err);
        mcpwm_del_operator(oper_); oper_ = nullptr;
        return err;
    }
    mcpwm_comparator_config_t cmpr_config = {}; cmpr_config.flags.update_cmp_on_tez = true;
    err = mcpwm_new_comparator(oper_, &cmpr_config, &cmpr_);
    ESP_LOGI(TAG, "[setup_pwm] mcpwm_new_comparator: cmpr_=%p, err=%d", cmpr_, err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建比较器失败，GPIO: %d, 错误: %d", gpio_, err);
        mcpwm_del_operator(oper_); oper_ = nullptr;
        return err;
    }
    mcpwm_generator_config_t gen_config = { .gen_gpio_num = gpio_ };
    err = mcpwm_new_generator(oper_, &gen_config, &gen_);
    ESP_LOGI(TAG, "[setup_pwm] mcpwm_new_generator: gen_=%p, err=%d", gen_, err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建 GPIO 生成器失败，GPIO: %d, 错误: %d", gpio_, err);
        mcpwm_del_comparator(cmpr_); mcpwm_del_operator(oper_); cmpr_ = nullptr; oper_ = nullptr;
        return err;
    }
    err = mcpwm_comparator_set_compare_value(cmpr_, pwm_range_.stop_pulse_width_us);
    ESP_LOGI(TAG, "[setup_pwm] mcpwm_comparator_set_compare_value: cmpr_=%p, value=%lu, err=%d", cmpr_, pwm_range_.stop_pulse_width_us, err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置比较器值失败，GPIO: %d, 错误: %d", gpio_, err);
        mcpwm_del_generator(gen_); mcpwm_del_comparator(cmpr_); mcpwm_del_operator(oper_); gen_ = nullptr; cmpr_ = nullptr; oper_ = nullptr;
        return err;
    }
    err = mcpwm_generator_set_action_on_timer_event(gen_, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    ESP_LOGI(TAG, "[setup_pwm] mcpwm_generator_set_action_on_timer_event: gen_=%p, err=%d", gen_, err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置定时器事件动作失败，GPIO: %d, 错误: %d", gpio_, err);
        mcpwm_del_generator(gen_); mcpwm_del_comparator(cmpr_); mcpwm_del_operator(oper_); gen_ = nullptr; cmpr_ = nullptr; oper_ = nullptr;
        return err;
    }
    err = mcpwm_generator_set_action_on_compare_event(gen_, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_, MCPWM_GEN_ACTION_LOW));
    ESP_LOGI(TAG, "[setup_pwm] mcpwm_generator_set_action_on_compare_event: gen_=%p, cmpr_=%p, err=%d", gen_, cmpr_, err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置比较器事件动作失败，GPIO: %d, 错误: %d", gpio_, err);
        mcpwm_del_generator(gen_); mcpwm_del_comparator(cmpr_); mcpwm_del_operator(oper_); gen_ = nullptr; cmpr_ = nullptr; oper_ = nullptr;
        return err;
    }
    ESP_LOGI(TAG, "MCPWM 设置完成，GPIO: %d, oper_=%p, cmpr_=%p, gen_=%p", gpio_, oper_, cmpr_, gen_);
    return ESP_OK;
}

void Servo360::set_speed(int speed, int duration_ms) {
    ESP_LOGI(TAG, "set_speed: gpio=%d, initialized_=%d, reverse=%d, speed=%d, duration_ms=%d", gpio_, initialized_, reverse_, speed, duration_ms);
    if (!initialized_) {
        ESP_LOGW(TAG, "舵机未初始化，无法设置速度: gpio=%d", gpio_);
        return;
    }
    int orig_speed = speed;
    if (reverse_) speed = -speed;
    ESP_LOGI(TAG, "set_speed: gpio=%d, reverse=%d, input_speed=%d, actual_speed=%d, duration_ms=%d", gpio_, reverse_, orig_speed, speed, duration_ms);
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    if (abs(speed - current_speed_) < 3) {
        ESP_LOGD(TAG, "速度变化太小，忽略设置: 当前=%d, 目标=%d", current_speed_, speed);
        return;
    }
    current_speed_ = speed;
    set_speed_internal(speed);
}

void Servo360::set_speed_internal(int speed) {
    uint32_t pulse_width_us;
    if (speed == 0) {
        pulse_width_us = pwm_range_.stop_pulse_width_us;
    } else if (speed > 0) {
        pulse_width_us = pwm_range_.stop_pulse_width_us + (pwm_range_.max_fwd_pulse_width_us - pwm_range_.stop_pulse_width_us) * speed / 100;
    } else {
        pulse_width_us = pwm_range_.stop_pulse_width_us - (pwm_range_.stop_pulse_width_us - pwm_range_.max_rev_pulse_width_us) * (-speed) / 100;
        ESP_LOGI(TAG, "set_speed_internal: 负速度分支, gpio=%d, speed=%d, pulse_width_us=%lu", gpio_, speed, pulse_width_us);
    }
    ESP_LOGI(TAG, "set_speed_internal: gpio=%d, speed=%d, pulse_width_us=%lu, oper_=%p, cmpr_=%p, gen_=%p", gpio_, speed, pulse_width_us, oper_, cmpr_, gen_);
    esp_err_t err = mcpwm_comparator_set_compare_value(cmpr_, pulse_width_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置比较器值失败: gpio=%d, err=%d, pulse_width_us=%lu", gpio_, err, pulse_width_us);
        return;
    }
}

void Servo360::stop() {
    if (!initialized_) {
        return;
    }
    ESP_LOGD(TAG, "停止舵机");
    set_speed(0);
}

void Servo360::run_for(int speed, int duration_ms) {
    if (!initialized_) {
        ESP_LOGW(TAG, "舵机未初始化，无法执行动作");
        return;
    }
    
    if (duration_ms <= 0) {
        ESP_LOGW(TAG, "持续时间无效: %d ms", duration_ms);
        return;
    }
    
    ESP_LOGI(TAG, "run_for: gpio=%d, speed=%d, duration_ms=%d", gpio_, speed, duration_ms);
    
    // 连续旋转舵机的速度范围：-100 到 100
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    
    // 连续旋转舵机的最小速度阈值
    if (abs(speed) < 5) {
        speed = (speed > 0) ? 5 : -5;
        ESP_LOGI(TAG, "速度太小，调整为: %d", speed);
    }
    
    // 立即设置速度
    set_speed(speed);
    
    // 连续旋转舵机不需要频繁更新，使用较长的延迟间隔
    int delay_interval = 100; // 100ms 间隔
    int total_delays = duration_ms / delay_interval;
    
    for (int i = 0; i < total_delays; ++i) {
        vTaskDelay(pdMS_TO_TICKS(delay_interval));
    }
    
    // 处理剩余时间
    int remaining_ms = duration_ms % delay_interval;
    if (remaining_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(remaining_ms));
    }
    
    // 确保停止
    stop();
    ESP_LOGI(TAG, "run_for 完成: gpio=%d", gpio_);
}

void Servo360::quick_action(int speed, int duration_ms) {
    if (!initialized_) {
        ESP_LOGW(TAG, "舵机未初始化，无法执行快速动作");
        return;
    }
    
    if (duration_ms <= 0) {
        ESP_LOGW(TAG, "持续时间无效: %d ms", duration_ms);
        return;
    }
    
    ESP_LOGI(TAG, "quick_action: gpio=%d, speed=%d, duration_ms=%d", gpio_, speed, duration_ms);
    
    // 立即设置速度
    set_speed(speed);
    
    // 创建任务来处理延迟和停止，避免阻塞主线程
    xTaskCreate([](void* param) {
        auto* servo = static_cast<Servo360*>(param);
        vTaskDelay(pdMS_TO_TICKS(100)); // 短暂延迟确保速度设置完成
        servo->stop();
        vTaskDelete(nullptr);
    }, "quick_action", 2048, this, 5, nullptr);
}

// 复合动作举例
void Servo360::raise_arm(int speed, int duration_ms) {
    if (!initialized_) {
        ESP_LOGW(TAG, "舵机未初始化，无法执行举手动作");
        return;
    }
    ESP_LOGI(TAG, "raise_arm: gpio=%d, speed=%d, duration_ms=%d", gpio_, speed, duration_ms);
    run_for(speed, duration_ms);
}

void Servo360::wave(int speed, int duration_ms, int count) {
    ESP_LOGI(TAG, "wave: gpio=%d, initialized_=%d, speed=%d, duration_ms=%d, count=%d", gpio_, initialized_, speed, duration_ms, count);
    if (!initialized_ || speed == 0 || duration_ms <= 0 || count <= 0) {
        ESP_LOGW(TAG, "wave参数无效: gpio=%d, speed=%d, duration_ms=%d, count=%d", gpio_, speed, duration_ms, count);
        return;
    }
    for (int i = 0; i < count; ++i) {
        ESP_LOGI(TAG, "wave: 正向, gpio=%d, speed=%d", gpio_, speed);
        run_for(speed, duration_ms);
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "wave: 反向, gpio=%d, speed=%d", gpio_, -speed);
        run_for(-speed, duration_ms);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    stop();
}

void Servo360::salute(int speed, int duration_ms) {
    if (!initialized_) {
        ESP_LOGW(TAG, "舵机未初始化，无法执行敬礼动作");
        return;
    }
    ESP_LOGI(TAG, "salute: gpio=%d, speed=%d, duration_ms=%d", gpio_, speed, duration_ms);
    run_for(speed, duration_ms);
    stop();
}

int Servo360::get_current_speed() const {
    return current_speed_;
}

gpio_num_t Servo360::get_gpio() const {
    return gpio_;
}

void Servo360::back_and_forth(int speed, int duration_ms, int count) {
    ESP_LOGI(TAG, "back_and_forth: gpio=%d, initialized_=%d, speed=%d, duration_ms=%d, count=%d", gpio_, initialized_, speed, duration_ms, count);
    if (!initialized_ || speed == 0 || duration_ms <= 0 || count <= 0) {
        ESP_LOGW(TAG, "back_and_forth参数无效: gpio=%d, speed=%d, duration_ms=%d, count=%d", gpio_, speed, duration_ms, count);
        return;
    }
    for (int i = 0; i < count; ++i) {
        ESP_LOGI(TAG, "back_and_forth: 正向, gpio=%d, speed=%d", gpio_, speed);
        set_speed(speed);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        ESP_LOGI(TAG, "back_and_forth: 反向, gpio=%d, speed=%d", gpio_, -speed);
        set_speed(-speed);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
    stop();
}

void Servo360::set_raw_pulse_width(uint32_t pulse_width_us) {
    if (!initialized_) {
        ESP_LOGW(TAG, "舵机未初始化，无法设置脉冲宽度");
        return;
    }
    
    // 验证脉冲宽度范围
    if (pulse_width_us < 1000 || pulse_width_us > 2000) {
        ESP_LOGW(TAG, "脉冲宽度超出范围: %lu us (应在 1000-2000 之间)", pulse_width_us);
        return;
    }
    
    ESP_LOGI(TAG, "设置原始脉冲宽度: %lu us, GPIO: %d", pulse_width_us, gpio_);
    
    // 直接设置比较器值
    esp_err_t err = mcpwm_comparator_set_compare_value(cmpr_, pulse_width_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置比较器值失败: %d, 脉冲宽度: %lu us", err, pulse_width_us);
        return;
    }
    
    // 更新当前速度（估算）
    if (pulse_width_us == pwm_range_.stop_pulse_width_us) {
        current_speed_ = 0;
    } else if (pulse_width_us > pwm_range_.stop_pulse_width_us) {
        // 正向
        current_speed_ = (pulse_width_us - pwm_range_.stop_pulse_width_us) * 100 / 
                        (pwm_range_.max_fwd_pulse_width_us - pwm_range_.stop_pulse_width_us);
    } else {
        // 反向
        current_speed_ = -(pwm_range_.stop_pulse_width_us - pulse_width_us) * 100 / 
                        (pwm_range_.stop_pulse_width_us - pwm_range_.max_rev_pulse_width_us);
    }
    
    ESP_LOGI(TAG, "原始脉冲宽度设置完成: %lu us, 估算速度: %d", pulse_width_us, current_speed_);
}