#include "servo_360.h"
#include "mcp_server.h"
#include "servo_degree_mapper.h"
#include <cJSON.h>
#include <string>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"

// 工具函数，安全获取属性值（上移到最顶部）
template<typename T>
T get_property_or(const PropertyList& properties, const std::string& key, const T& default_value) {
    try {
        return properties[key].value<T>();
    } catch (...) {
        return default_value;
    }
}

#define TAG "ServoMCP"

// =====================
// 示例：角度指令解析层调用
// =====================
// 语音/文本解析后，假设得到目标臂、角度
// 可直接调用如下函数实现角度到协议参数的自动映射和下发：
// HandleArmDegreeCommand("left", 90); // 左手抬到90度
// HandleArmDegreeCommand("right", 180); // 右手抬到180度

static std::string normalize_target(const std::string& target) {
    if (target == "both_arms" || target == "arm" || target == "both" || target == "hand" || target == "both_hands") return "both";
    if (target == "right_arm" || target == "right" || target == "right_hand") return "right";
    if (target == "left_arm" || target == "left" || target == "left_hand") return "left";
    return target;
}

static bool is_valid_target(const std::string& target) {
    std::string t = normalize_target(target);
    return t == "left" || t == "right" || t == "both";
}

class ServoMcpController {
public:
    ServoMcpController(gpio_num_t left_gpio, gpio_num_t right_gpio)
        : shared_timer_(nullptr), left_servo_(nullptr), right_servo_(nullptr), initialized_(false) {
        
        // 创建共享的 MCPWM 定时器
        mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000, // 1MHz 分辨率，1us 精度
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = 20000,    // 20ms 周期 (50Hz)
        };
        esp_err_t err = mcpwm_new_timer(&timer_config, &shared_timer_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "创建 MCPWM 定时器失败: %d", err);
            return;
        }
        
        // 启动定时器
        err = mcpwm_timer_enable(shared_timer_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "启动 MCPWM 定时器失败: %d", err);
            mcpwm_del_timer(shared_timer_);
            shared_timer_ = nullptr;
            return;
        }
        
        err = mcpwm_timer_start_stop(shared_timer_, MCPWM_TIMER_START_NO_STOP);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "启动 MCPWM 定时器失败: %d", err);
            mcpwm_timer_disable(shared_timer_);
            mcpwm_del_timer(shared_timer_);
            shared_timer_ = nullptr;
            return;
        }
        
        ESP_LOGI(TAG, "MCPWM 定时器初始化成功，频率: 50Hz");
        
        // 创建舵机对象
        ESP_LOGI(TAG, "创建左臂舵机，GPIO: %d", left_gpio);
        left_servo_ = new Servo360(left_gpio, shared_timer_, nullptr, false);  // 左臂不反转
        ESP_LOGI(TAG, "创建右臂舵机，GPIO: %d", right_gpio);
        right_servo_ = new Servo360(right_gpio, shared_timer_, nullptr, false); // 右臂也不反转，通过软件控制方向
        
        if (left_servo_ && right_servo_) {
            initialized_ = true;
            ESP_LOGI(TAG, "舵机控制器初始化成功 - 左臂GPIO: %d, 右臂GPIO: %d", left_gpio, right_gpio);
        } else {
            ESP_LOGE(TAG, "舵机对象创建失败 - 左臂: %p, 右臂: %p", left_servo_, right_servo_);
            return;
        }
        
        auto& mcp_server = McpServer::GetInstance();

        // set
        mcp_server.AddTool("self.servo360.set",
            "控制舵机动作",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                int speed = properties["speed"].value<int>();
                int duration = properties["duration"].value<int>();
                if (!is_valid_target(target) || duration <= 0) {
                    ESP_LOGW(TAG, "参数无效: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                    return false;
                }
                ESP_LOGI(TAG, "Servo set: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                
                // 使用 run_for 方法，它会处理定时停止
                if (target == "left") {
                    left_servo_->run_for(speed, duration);
                } else if (target == "right") {
                    right_servo_->run_for(speed, duration);
                } else if (target == "both") {
                    // 并行执行双臂动作 - 为每个舵机创建独立任务
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "双臂并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->run_for(speed, duration);
                        ESP_LOGI(TAG, "双臂并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "servo_left", 4096, new std::tuple<Servo360*, int, int>(left_servo_, speed, duration), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "双臂并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->run_for(speed, duration);
                        ESP_LOGI(TAG, "双臂并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "servo_right", 4096, new std::tuple<Servo360*, int, int>(right_servo_, speed, duration), 5, nullptr);
                }
                return true;
            });

        // quick_set - 快速响应版本
        mcp_server.AddTool("self.servo360.quick_set",
            "快速控制舵机动作（非阻塞）",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                int speed = properties["speed"].value<int>();
                int duration = get_property_or<int>(properties, "duration", 100);
                if (!is_valid_target(target)) {
                    ESP_LOGW(TAG, "参数无效: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                    return false;
                }
                ESP_LOGI(TAG, "Servo quick_set: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                
                // 使用快速响应方法，非阻塞
                if (target == "left") {
                    left_servo_->quick_action(speed, duration);
                } else if (target == "right") {
                    right_servo_->quick_action(speed, duration);
                } else if (target == "both") {
                    left_servo_->quick_action(speed, duration);
                    right_servo_->quick_action(speed, duration);
                }
                return true;
            });

        // wave
        mcp_server.AddTool("self.servo360.wave",
            "舵机挥手动作",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("count", kPropertyTypeInteger),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                int count = get_property_or<int>(properties, "count", 3);
                int speed = get_property_or<int>(properties, "speed", 80);
                int duration = get_property_or<int>(properties, "duration", 400);
                if (!is_valid_target(target) || count <= 0 || duration <= 0) {
                    ESP_LOGW(TAG, "参数无效: target=%s count=%d speed=%d duration=%d", target.c_str(), count, speed, duration);
                    return false;
                }
                ESP_LOGI(TAG, "Servo wave: target=%s count=%d speed=%d duration=%d", target.c_str(), count, speed, duration);
                
                if (target == "left") {
                    left_servo_->wave(speed, duration, count);
                } else if (target == "right") {
                    right_servo_->wave(speed, duration, count);
                } else if (target == "both") {
                    // 并行执行双臂挥手 - 为每个舵机创建独立任务
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        int count = std::get<3>(*data);
                        
                        ESP_LOGI(TAG, "双臂挥手并行任务启动: GPIO=%d, speed=%d, duration=%d, count=%d", servo->get_gpio(), speed, duration, count);
                        servo->wave(speed, duration, count);
                        ESP_LOGI(TAG, "双臂挥手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "servo_wave_left", 4096, new std::tuple<Servo360*, int, int, int>(left_servo_, speed, duration, count), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        int count = std::get<3>(*data);
                        
                        ESP_LOGI(TAG, "双臂挥手并行任务启动: GPIO=%d, speed=%d, duration=%d, count=%d", servo->get_gpio(), speed, duration, count);
                        servo->wave(speed, duration, count);
                        ESP_LOGI(TAG, "双臂挥手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "servo_wave_right", 4096, new std::tuple<Servo360*, int, int, int>(right_servo_, speed, duration, count), 5, nullptr);
                }
                return true;
            });

        // raise
        mcp_server.AddTool("self.servo360.raise",
            "舵机举手动作",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                int speed = get_property_or<int>(properties, "speed", 80);
                int duration = get_property_or<int>(properties, "duration", 600);
                if (!is_valid_target(target) || duration <= 0) {
                    ESP_LOGW(TAG, "参数无效: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                    return false;
                }
                ESP_LOGI(TAG, "Servo raise: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                
                if (target == "left") {
                    left_servo_->raise_arm(speed, duration);
                } else if (target == "right") {
                    right_servo_->raise_arm(speed, duration);
                } else if (target == "both") {
                    // 并行执行双臂举手 - 为每个舵机创建独立任务
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "双臂举手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->raise_arm(speed, duration);
                        ESP_LOGI(TAG, "双臂举手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "servo_raise_left", 4096, new std::tuple<Servo360*, int, int>(left_servo_, speed, duration), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "双臂举手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->raise_arm(speed, duration);
                        ESP_LOGI(TAG, "双臂举手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "servo_raise_right", 4096, new std::tuple<Servo360*, int, int>(right_servo_, speed, duration), 5, nullptr);
                }
                return true;
            });

        // salute
        mcp_server.AddTool("self.servo360.salute",
            "舵机敬礼动作",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                int speed = get_property_or<int>(properties, "speed", 80);
                int duration = get_property_or<int>(properties, "duration", 500);
                if (!is_valid_target(target) || duration <= 0) {
                    ESP_LOGW(TAG, "参数无效: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                    return false;
                }
                ESP_LOGI(TAG, "Servo salute: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                
                if (target == "left") {
                    left_servo_->salute(speed, duration);
                } else if (target == "right") {
                    right_servo_->salute(speed, duration);
                } else if (target == "both") {
                    // 并行执行双臂敬礼 - 为每个舵机创建独立任务
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "双臂敬礼并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->salute(speed, duration);
                        ESP_LOGI(TAG, "双臂敬礼并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "servo_salute_left", 4096, new std::tuple<Servo360*, int, int>(left_servo_, speed, duration), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "双臂敬礼并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->salute(speed, duration);
                        ESP_LOGI(TAG, "双臂敬礼并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "servo_salute_right", 4096, new std::tuple<Servo360*, int, int>(right_servo_, speed, duration), 5, nullptr);
                }
                return true;
            });

        // 高级双臂协调动作
        mcp_server.AddTool("self.servo360.combo",
            "双臂组合动作：左手举手右手挥手",
            PropertyList({
                Property("action", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string action = properties["action"].value<std::string>();
                int speed = get_property_or<int>(properties, "speed", 80);
                int duration = get_property_or<int>(properties, "duration", 500);
                
                ESP_LOGI(TAG, "Servo combo: action=%s speed=%d duration=%d", action.c_str(), speed, duration);
                
                if (action == "raise_wave" || action == "举手挥手" || action == "combo") {
                    // 左手举手，右手挥手 - 并行执行
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "combo举手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->raise_arm(speed, duration);
                        ESP_LOGI(TAG, "combo举手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_raise", 4096, new std::tuple<Servo360*, int, int>(left_servo_, speed, duration), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        int count = std::get<3>(*data);
                        
                        ESP_LOGI(TAG, "combo挥手并行任务启动: GPIO=%d, speed=%d, duration=%d, count=%d", servo->get_gpio(), speed, duration, count);
                        servo->wave(speed, duration, count);
                        ESP_LOGI(TAG, "combo挥手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_wave", 4096, new std::tuple<Servo360*, int, int, int>(right_servo_, speed, duration/2, 2), 5, nullptr);
                } else if (action == "wave_raise" || action == "挥手举手") {
                    // 左手挥手，右手举手 - 并行执行
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        int count = std::get<3>(*data);
                        
                        ESP_LOGI(TAG, "combo挥手并行任务启动: GPIO=%d, speed=%d, duration=%d, count=%d", servo->get_gpio(), speed, duration, count);
                        servo->wave(speed, duration, count);
                        ESP_LOGI(TAG, "combo挥手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_wave", 4096, new std::tuple<Servo360*, int, int, int>(left_servo_, speed, duration/2, 2), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "combo举手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->raise_arm(speed, duration);
                        ESP_LOGI(TAG, "combo举手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_raise", 4096, new std::tuple<Servo360*, int, int>(right_servo_, speed, duration), 5, nullptr);
                } else if (action == "wave" || action == "挥手") {
                    // 双臂同时挥手 - 并行执行
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        int count = std::get<3>(*data);
                        
                        ESP_LOGI(TAG, "combo双臂挥手并行任务启动: GPIO=%d, speed=%d, duration=%d, count=%d", servo->get_gpio(), speed, duration, count);
                        servo->wave(speed, duration, count);
                        ESP_LOGI(TAG, "combo双臂挥手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_wave_left", 4096, new std::tuple<Servo360*, int, int, int>(left_servo_, speed, duration/2, 2), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        int count = std::get<3>(*data);
                        
                        ESP_LOGI(TAG, "combo双臂挥手并行任务启动: GPIO=%d, speed=%d, duration=%d, count=%d", servo->get_gpio(), speed, duration, count);
                        servo->wave(speed, duration, count);
                        ESP_LOGI(TAG, "combo双臂挥手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_wave_right", 4096, new std::tuple<Servo360*, int, int, int>(right_servo_, speed, duration/2, 2), 5, nullptr);
                } else if (action == "raise" || action == "举手") {
                    // 双臂同时举手 - 并行执行
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "combo双臂举手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->raise_arm(speed, duration);
                        ESP_LOGI(TAG, "combo双臂举手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_raise_left", 4096, new std::tuple<Servo360*, int, int>(left_servo_, speed, duration), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "combo双臂举手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->raise_arm(speed, duration);
                        ESP_LOGI(TAG, "combo双臂举手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_raise_right", 4096, new std::tuple<Servo360*, int, int>(right_servo_, speed, duration), 5, nullptr);
                } else if (action == "salute" || action == "敬礼") {
                    // 双臂同时敬礼 - 并行执行
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "combo双臂敬礼并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->salute(speed, duration);
                        ESP_LOGI(TAG, "combo双臂敬礼并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_salute_left", 4096, new std::tuple<Servo360*, int, int>(left_servo_, speed, duration), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "combo双臂敬礼并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->salute(speed, duration);
                        ESP_LOGI(TAG, "combo双臂敬礼并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "combo_salute_right", 4096, new std::tuple<Servo360*, int, int>(right_servo_, speed, duration), 5, nullptr);
                } else {
                    ESP_LOGW(TAG, "未知组合动作: %s", action.c_str());
                    return false;
                }
                return true;
            });

        // 双臂交替动作
        mcp_server.AddTool("self.servo360.alternate",
            "双臂交替动作",
            PropertyList({
                Property("action", kPropertyTypeString),
                Property("count", kPropertyTypeInteger),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string action = properties["action"].value<std::string>();
                int count = get_property_or<int>(properties, "count", 3);
                int speed = get_property_or<int>(properties, "speed", 80);
                int duration = get_property_or<int>(properties, "duration", 300);
                
                ESP_LOGI(TAG, "Servo alternate: action=%s count=%d speed=%d duration=%d", action.c_str(), count, speed, duration);
                
                for (int i = 0; i < count; ++i) {
                    if (action == "wave" || action == "挥手") {
                        // 交替挥手 - 使用 run_for 方法
                        left_servo_->run_for(speed, duration);
                        left_servo_->run_for(-speed, duration);
                        
                        right_servo_->run_for(speed, duration);
                        right_servo_->run_for(-speed, duration);
                    } else if (action == "raise" || action == "举手") {
                        // 交替举手
                        left_servo_->raise_arm(speed, duration);
                        vTaskDelay(pdMS_TO_TICKS(duration));
                        right_servo_->raise_arm(speed, duration);
                        vTaskDelay(pdMS_TO_TICKS(duration));
                    } else {
                        ESP_LOGW(TAG, "未知交替动作: %s", action.c_str());
                        return false;
                    }
                }
                return true;
            });

        // 双臂镜像动作
        mcp_server.AddTool("self.servo360.mirror",
            "双臂镜像动作（左右对称）",
            PropertyList({
                Property("action", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string action = properties["action"].value<std::string>();
                int speed = get_property_or<int>(properties, "speed", 80);
                int duration = get_property_or<int>(properties, "duration", 500);
                
                ESP_LOGI(TAG, "Servo mirror: action=%s speed=%d duration=%d", action.c_str(), speed, duration);
                
                if (action == "wave" || action == "挥手") {
                    // 镜像挥手（左右相反方向）- 并行执行
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "mirror挥手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        // 执行3次来回动作
                        for (int i = 0; i < 3; ++i) {
                            servo->run_for(speed, duration);
                            servo->run_for(-speed, duration);
                        }
                        ESP_LOGI(TAG, "mirror挥手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "mirror_wave_left", 4096, new std::tuple<Servo360*, int, int>(left_servo_, speed, duration), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "mirror挥手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        // 执行3次来回动作（相反方向）
                        for (int i = 0; i < 3; ++i) {
                            servo->run_for(-speed, duration);
                            servo->run_for(speed, duration);
                        }
                        ESP_LOGI(TAG, "mirror挥手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "mirror_wave_right", 4096, new std::tuple<Servo360*, int, int>(right_servo_, speed, duration), 5, nullptr);
                } else if (action == "raise" || action == "举手") {
                    // 镜像举手（左右同时）- 并行执行
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "mirror举手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->raise_arm(speed, duration);
                        ESP_LOGI(TAG, "mirror举手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "mirror_raise_left", 4096, new std::tuple<Servo360*, int, int>(left_servo_, speed, duration), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "mirror举手并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->raise_arm(speed, duration);
                        ESP_LOGI(TAG, "mirror举手并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "mirror_raise_right", 4096, new std::tuple<Servo360*, int, int>(right_servo_, speed, duration), 5, nullptr);
                } else if (action == "salute" || action == "敬礼") {
                    // 镜像敬礼（左右同时）- 并行执行
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "mirror敬礼并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->salute(speed, duration);
                        ESP_LOGI(TAG, "mirror敬礼并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "mirror_salute_left", 4096, new std::tuple<Servo360*, int, int>(left_servo_, speed, duration), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        
                        ESP_LOGI(TAG, "mirror敬礼并行任务启动: GPIO=%d, speed=%d, duration=%d", servo->get_gpio(), speed, duration);
                        servo->salute(speed, duration);
                        ESP_LOGI(TAG, "mirror敬礼并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "mirror_salute_right", 4096, new std::tuple<Servo360*, int, int>(right_servo_, speed, duration), 5, nullptr);
                } else {
                    ESP_LOGW(TAG, "未知镜像动作: %s", action.c_str());
                    return false;
                }
                return true;
            });

        // back_and_forth - 来回动作测试
        mcp_server.AddTool("self.servo360.back_and_forth",
            "舵机来回动作测试",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger),
                Property("count", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                int speed = get_property_or<int>(properties, "speed", 80);
                int duration = get_property_or<int>(properties, "duration", 300);
                int count = get_property_or<int>(properties, "count", 2);
                
                if (!is_valid_target(target)) {
                    ESP_LOGW(TAG, "参数无效: target=%s speed=%d duration=%d count=%d", target.c_str(), speed, duration, count);
                    return false;
                }
                ESP_LOGI(TAG, "Servo back_and_forth: target=%s speed=%d duration=%d count=%d", target.c_str(), speed, duration, count);
                
                if (target == "left") {
                    left_servo_->back_and_forth(speed, duration, count);
                } else if (target == "right") {
                    right_servo_->back_and_forth(speed, duration, count);
                } else if (target == "both") {
                    // 并行执行双臂来回动作
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        int count = std::get<3>(*data);
                        
                        ESP_LOGI(TAG, "back_and_forth并行任务启动: GPIO=%d, speed=%d, duration=%d, count=%d", servo->get_gpio(), speed, duration, count);
                        servo->back_and_forth(speed, duration, count);
                        ESP_LOGI(TAG, "back_and_forth并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "back_forth_left", 4096, new std::tuple<Servo360*, int, int, int>(left_servo_, speed, duration, count), 5, nullptr);
                    
                    xTaskCreate([](void* param) {
                        auto* data = static_cast<std::tuple<Servo360*, int, int, int>*>(param);
                        auto* servo = std::get<0>(*data);
                        int speed = std::get<1>(*data);
                        int duration = std::get<2>(*data);
                        int count = std::get<3>(*data);
                        
                        ESP_LOGI(TAG, "back_and_forth并行任务启动: GPIO=%d, speed=%d, duration=%d, count=%d", servo->get_gpio(), speed, duration, count);
                        servo->back_and_forth(speed, duration, count);
                        ESP_LOGI(TAG, "back_and_forth并行任务完成: GPIO=%d", servo->get_gpio());
                        
                        delete data;
                        vTaskDelete(nullptr);
                    }, "back_forth_right", 4096, new std::tuple<Servo360*, int, int, int>(right_servo_, speed, duration, count), 5, nullptr);
                }
                return true;
            });

        // query
        mcp_server.AddTool("self.servo360.query",
            "查询舵机状态",
            PropertyList({
                Property("target", kPropertyTypeString)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                if (!is_valid_target(target)) {
                    ESP_LOGW(TAG, "参数无效: target=%s", target.c_str());
                    return false;
                }
                
                if (target == "left") {
                    ESP_LOGI(TAG, "左臂舵机状态: GPIO=%d, 当前速度=%d", 
                             left_servo_->get_gpio(), left_servo_->get_current_speed());
                } else if (target == "right") {
                    ESP_LOGI(TAG, "右臂舵机状态: GPIO=%d, 当前速度=%d", 
                             right_servo_->get_gpio(), right_servo_->get_current_speed());
                } else if (target == "both") {
                    ESP_LOGI(TAG, "舵机状态 - 左臂: GPIO=%d, 速度=%d; 右臂: GPIO=%d, 速度=%d", 
                             left_servo_->get_gpio(), left_servo_->get_current_speed(),
                             right_servo_->get_gpio(), right_servo_->get_current_speed());
                }
                return true;
            });

        // test_direction - 测试舵机方向
        mcp_server.AddTool("self.servo360.test_direction",
            "测试舵机正向和反向运动",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger),
                Property("duration", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                int speed = get_property_or<int>(properties, "speed", 50);
                int duration = get_property_or<int>(properties, "duration", 500);
                if (!is_valid_target(target) || duration <= 0) {
                    ESP_LOGW(TAG, "参数无效: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                    return false;
                }
                ESP_LOGI(TAG, "测试舵机方向: target=%s speed=%d duration=%d", target.c_str(), speed, duration);
                
                // 创建测试任务
                xTaskCreate([](void* param) {
                    auto* data = static_cast<std::tuple<Servo360*, Servo360*, std::string, int, int>*>(param);
                    auto* left_servo = std::get<0>(*data);
                    auto* right_servo = std::get<1>(*data);
                    std::string target = std::get<2>(*data);
                    int speed = std::get<3>(*data);
                    int duration = std::get<4>(*data);
                    
                    if (target == "left") {
                        ESP_LOGI(TAG, "测试左臂舵机 - 正向运动");
                        left_servo->run_for(speed, duration);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        ESP_LOGI(TAG, "测试左臂舵机 - 反向运动");
                        left_servo->run_for(-speed, duration);
                    } else if (target == "right") {
                        ESP_LOGI(TAG, "测试右臂舵机 - 正向运动");
                        right_servo->run_for(speed, duration);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        ESP_LOGI(TAG, "测试右臂舵机 - 反向运动");
                        right_servo->run_for(-speed, duration);
                    } else if (target == "both") {
                        ESP_LOGI(TAG, "测试双臂舵机 - 正向运动");
                        left_servo->run_for(speed, duration);
                        right_servo->run_for(speed, duration);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        ESP_LOGI(TAG, "测试双臂舵机 - 反向运动");
                        left_servo->run_for(-speed, duration);
                        right_servo->run_for(-speed, duration);
                    }
                    
                    delete data;
                    vTaskDelete(nullptr);
                }, "test_direction", 4096, new std::tuple<Servo360*, Servo360*, std::string, int, int>(left_servo_, right_servo_, target, speed, duration), 5, nullptr);
                
                return true;
            });

        // calibrate - 舵机校准工具
        mcp_server.AddTool("self.servo360.calibrate",
            "舵机校准工具",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("pulse_width", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                int pulse_width = get_property_or<int>(properties, "pulse_width", 1500);
                if (!is_valid_target(target) || pulse_width < 1000 || pulse_width > 2000) {
                    ESP_LOGW(TAG, "参数无效: target=%s pulse_width=%d", target.c_str(), pulse_width);
                    return false;
                }
                ESP_LOGI(TAG, "舵机校准: target=%s pulse_width=%d", target.c_str(), pulse_width);
                
                // 直接设置 PWM 脉冲宽度进行测试
                if (target == "left") {
                    left_servo_->set_raw_pulse_width(pulse_width);
                } else if (target == "right") {
                    right_servo_->set_raw_pulse_width(pulse_width);
                } else if (target == "both") {
                    left_servo_->set_raw_pulse_width(pulse_width);
                    right_servo_->set_raw_pulse_width(pulse_width);
                }
                return true;
            });

        // anti_vibration_test - 防震动测试
        mcp_server.AddTool("self.servo360.anti_vibration_test",
            "防震动测试工具",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("test_type", kPropertyTypeString)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                std::string test_type = get_property_or<std::string>(properties, "test_type", "gradual");
                if (!is_valid_target(target)) {
                    ESP_LOGW(TAG, "参数无效: target=%s", target.c_str());
                    return false;
                }
                ESP_LOGI(TAG, "防震动测试: target=%s test_type=%s", target.c_str(), test_type.c_str());
                
                // 创建测试任务
                xTaskCreate([](void* param) {
                    auto* data = static_cast<std::tuple<Servo360*, Servo360*, std::string, std::string>*>(param);
                    auto* left_servo = std::get<0>(*data);
                    auto* right_servo = std::get<1>(*data);
                    std::string target = std::get<2>(*data);
                    std::string test_type = std::get<3>(*data);
                    
                    if (test_type == "gradual") {
                        // 渐进式测试：从小速度开始
                        ESP_LOGI(TAG, "渐进式防震动测试开始");
                        for (int speed = 10; speed <= 50; speed += 10) {
                            ESP_LOGI(TAG, "测试速度: %d", speed);
                            if (target == "left" || target == "both") {
                                left_servo->run_for(speed, 1000);
                            }
                            if (target == "right" || target == "both") {
                                right_servo->run_for(speed, 1000);
                            }
                            vTaskDelay(pdMS_TO_TICKS(500));
                            
                            // 反向测试
                            if (target == "left" || target == "both") {
                                left_servo->run_for(-speed, 1000);
                            }
                            if (target == "right" || target == "both") {
                                right_servo->run_for(-speed, 1000);
                            }
                            vTaskDelay(pdMS_TO_TICKS(500));
                        }
                    } else if (test_type == "pulse_width") {
                        // 脉冲宽度测试：测试不同的脉冲宽度
                        ESP_LOGI(TAG, "脉冲宽度测试开始");
                        uint32_t pulse_widths[] = {1500, 1550, 1600, 1650, 1700, 1750, 1800, 1450, 1400, 1350, 1300, 1250, 1200};
                        for (uint32_t pw : pulse_widths) {
                            ESP_LOGI(TAG, "测试脉冲宽度: %lu us", pw);
                            if (target == "left" || target == "both") {
                                left_servo->set_raw_pulse_width(pw);
                            }
                            if (target == "right" || target == "both") {
                                right_servo->set_raw_pulse_width(pw);
                            }
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        }
                        // 回到停止位置
                        if (target == "left" || target == "both") {
                            left_servo->stop();
                        }
                        if (target == "right" || target == "both") {
                            right_servo->stop();
                        }
                    } else if (test_type == "stability") {
                        // 稳定性测试：长时间运行
                        ESP_LOGI(TAG, "稳定性测试开始");
                        for (int i = 0; i < 5; ++i) {
                            ESP_LOGI(TAG, "稳定性测试轮次: %d", i + 1);
                            if (target == "left" || target == "both") {
                                left_servo->run_for(30, 2000);
                            }
                            if (target == "right" || target == "both") {
                                right_servo->run_for(30, 2000);
                            }
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        }
                    }
                    
                    ESP_LOGI(TAG, "防震动测试完成");
                    delete data;
                    vTaskDelete(nullptr);
                }, "anti_vibration_test", 4096, new std::tuple<Servo360*, Servo360*, std::string, std::string>(left_servo_, right_servo_, target, test_type), 5, nullptr);
                
                return true;
            });

        // power_check - 电源状态检查
        mcp_server.AddTool("self.servo360.power_check",
            "检查舵机电源状态",
            PropertyList({
                Property("target", kPropertyTypeString)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                if (!is_valid_target(target)) {
                    ESP_LOGW(TAG, "参数无效: target=%s", target.c_str());
                    return false;
                }
                ESP_LOGI(TAG, "电源状态检查: target=%s", target.c_str());
                
                // 创建电源检查任务
                xTaskCreate([](void* param) {
                    auto* data = static_cast<std::tuple<Servo360*, Servo360*, std::string>*>(param);
                    auto* left_servo = std::get<0>(*data);
                    auto* right_servo = std::get<1>(*data);
                    std::string target = std::get<2>(*data);
                    
                    ESP_LOGI(TAG, "开始电源状态检查...");
                    
                    // 测试小负载
                    ESP_LOGI(TAG, "测试小负载 (速度=10)...");
                    if (target == "left" || target == "both") {
                        left_servo->run_for(10, 500);
                    }
                    if (target == "right" || target == "both") {
                        right_servo->run_for(10, 500);
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    // 测试中等负载
                    ESP_LOGI(TAG, "测试中等负载 (速度=30)...");
                    if (target == "left" || target == "both") {
                        left_servo->run_for(30, 500);
                    }
                    if (target == "right" || target == "both") {
                        right_servo->run_for(30, 500);
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    // 测试大负载
                    ESP_LOGI(TAG, "测试大负载 (速度=50)...");
                    if (target == "left" || target == "both") {
                        left_servo->run_for(50, 500);
                    }
                    if (target == "right" || target == "both") {
                        right_servo->run_for(50, 500);
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    // 测试脉冲宽度稳定性
                    ESP_LOGI(TAG, "测试脉冲宽度稳定性...");
                    uint32_t test_pulse = 1600; // 中等正向速度
                    if (target == "left" || target == "both") {
                        left_servo->set_raw_pulse_width(test_pulse);
                    }
                    if (target == "right" || target == "both") {
                        right_servo->set_raw_pulse_width(test_pulse);
                    }
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    
                    // 停止所有舵机
                    if (target == "left" || target == "both") {
                        left_servo->stop();
                    }
                    if (target == "right" || target == "both") {
                        right_servo->stop();
                    }
                    
                    ESP_LOGI(TAG, "电源状态检查完成");
                    ESP_LOGI(TAG, "如果舵机在测试过程中出现震动或停止，可能是电源供电不足");
                    ESP_LOGI(TAG, "建议：1. 检查电源电压是否稳定 2. 确保电源能提供足够电流 3. 检查接线是否牢固");
                    
                    delete data;
                    vTaskDelete(nullptr);
                }, "power_check", 4096, new std::tuple<Servo360*, Servo360*, std::string>(left_servo_, right_servo_, target), 5, nullptr);
                
                return true;
            });

        // fine_tune - 微调工具
        mcp_server.AddTool("self.servo360.fine_tune",
            "微调舵机参数，找到最佳设置",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("direction", kPropertyTypeString),
                Property("start_pulse", kPropertyTypeInteger),
                Property("end_pulse", kPropertyTypeInteger),
                Property("step", kPropertyTypeInteger)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                std::string direction = get_property_or<std::string>(properties, "direction", "forward");
                int start_pulse = get_property_or<int>(properties, "start_pulse", 1500);
                int end_pulse = get_property_or<int>(properties, "end_pulse", 1700);
                int step = get_property_or<int>(properties, "step", 25);
                
                if (!is_valid_target(target) || start_pulse < 1000 || end_pulse > 2000 || step <= 0) {
                    ESP_LOGW(TAG, "参数无效: target=%s direction=%s start=%d end=%d step=%d", 
                             target.c_str(), direction.c_str(), start_pulse, end_pulse, step);
                    return false;
                }
                
                if (direction == "reverse") {
                    // 反向测试
                    int temp = start_pulse;
                    start_pulse = end_pulse;
                    end_pulse = temp;
                }
                
                ESP_LOGI(TAG, "微调测试: target=%s direction=%s start=%d end=%d step=%d", 
                         target.c_str(), direction.c_str(), start_pulse, end_pulse, step);
                
                // 创建微调任务
                xTaskCreate([](void* param) {
                    auto* data = static_cast<std::tuple<Servo360*, Servo360*, std::string, int, int, int>*>(param);
                    auto* left_servo = std::get<0>(*data);
                    auto* right_servo = std::get<1>(*data);
                    std::string target = std::get<2>(*data);
                    int start_pulse = std::get<3>(*data);
                    int end_pulse = std::get<4>(*data);
                    int step = std::get<5>(*data);
                    
                    ESP_LOGI(TAG, "开始微调测试...");
                    ESP_LOGI(TAG, "请观察舵机在每个脉冲宽度下的表现");
                    ESP_LOGI(TAG, "找到最稳定、无震动的脉冲宽度值");
                    
                    for (int pulse = start_pulse; pulse <= end_pulse; pulse += step) {
                        ESP_LOGI(TAG, "测试脉冲宽度: %d us", pulse);
                        
                        if (target == "left" || target == "both") {
                            left_servo->set_raw_pulse_width(pulse);
                        }
                        if (target == "right" || target == "both") {
                            right_servo->set_raw_pulse_width(pulse);
                        }
                        
                        // 每个脉冲宽度测试 2 秒
                        vTaskDelay(pdMS_TO_TICKS(2000));
                    }
                    
                    // 回到停止位置
                    ESP_LOGI(TAG, "微调测试完成，回到停止位置");
                    if (target == "left" || target == "both") {
                        left_servo->stop();
                    }
                    if (target == "right" || target == "both") {
                        right_servo->stop();
                    }
                    
                    ESP_LOGI(TAG, "微调测试完成");
                    ESP_LOGI(TAG, "请记录下最稳定的脉冲宽度值，可用于后续调整");
                    
                    delete data;
                    vTaskDelete(nullptr);
                }, "fine_tune", 4096, new std::tuple<Servo360*, Servo360*, std::string, int, int, int>(left_servo_, right_servo_, target, start_pulse, end_pulse, step), 5, nullptr);
                
                return true;
            });

        // continuous_rotation_test - 连续旋转舵机测试
        mcp_server.AddTool("self.servo360.continuous_rotation_test",
            "连续旋转舵机专用测试工具",
            PropertyList({
                Property("target", kPropertyTypeString),
                Property("test_type", kPropertyTypeString)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!initialized_) {
                    ESP_LOGW(TAG, "舵机控制器未初始化");
                    return false;
                }
                std::string target = normalize_target(properties["target"].value<std::string>());
                std::string test_type = get_property_or<std::string>(properties, "test_type", "speed_test");
                if (!is_valid_target(target)) {
                    ESP_LOGW(TAG, "参数无效: target=%s", target.c_str());
                    return false;
                }
                ESP_LOGI(TAG, "连续旋转舵机测试: target=%s test_type=%s", target.c_str(), test_type.c_str());
                
                // 创建测试任务
                xTaskCreate([](void* param) {
                    auto* data = static_cast<std::tuple<Servo360*, Servo360*, std::string, std::string>*>(param);
                    auto* left_servo = std::get<0>(*data);
                    auto* right_servo = std::get<1>(*data);
                    std::string target = std::get<2>(*data);
                    std::string test_type = std::get<3>(*data);
                    
                    if (test_type == "speed_test") {
                        // 速度测试：测试不同速度
                        ESP_LOGI(TAG, "连续旋转舵机速度测试开始");
                        int speeds[] = {10, 20, 30, 50, 70, 100};
                        for (int speed : speeds) {
                            ESP_LOGI(TAG, "测试正向速度: %d", speed);
                            if (target == "left" || target == "both") {
                                left_servo->run_for(speed, 2000);
                            }
                            if (target == "right" || target == "both") {
                                right_servo->run_for(speed, 2000);
                            }
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            
                            ESP_LOGI(TAG, "测试反向速度: %d", -speed);
                            if (target == "left" || target == "both") {
                                left_servo->run_for(-speed, 2000);
                            }
                            if (target == "right" || target == "both") {
                                right_servo->run_for(-speed, 2000);
                            }
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        }
                    } else if (test_type == "pulse_test") {
                        // 脉冲宽度测试：测试连续旋转舵机的脉冲宽度范围
                        ESP_LOGI(TAG, "连续旋转舵机脉冲宽度测试开始");
                        uint32_t pulse_widths[] = {1500, 1600, 1700, 1800, 1900, 2000, 1400, 1300, 1200, 1100, 1000};
                        for (uint32_t pw : pulse_widths) {
                            ESP_LOGI(TAG, "测试脉冲宽度: %lu us", pw);
                            if (target == "left" || target == "both") {
                                left_servo->set_raw_pulse_width(pw);
                            }
                            if (target == "right" || target == "both") {
                                right_servo->set_raw_pulse_width(pw);
                            }
                            vTaskDelay(pdMS_TO_TICKS(2000));
                        }
                    } else if (test_type == "continuous_test") {
                        // 连续运行测试
                        ESP_LOGI(TAG, "连续旋转舵机连续运行测试开始");
                        for (int i = 0; i < 3; ++i) {
                            ESP_LOGI(TAG, "连续运行测试轮次: %d", i + 1);
                            if (target == "left" || target == "both") {
                                left_servo->run_for(50, 5000);
                            }
                            if (target == "right" || target == "both") {
                                right_servo->run_for(50, 5000);
                            }
                            vTaskDelay(pdMS_TO_TICKS(2000));
                        }
                    }
                    
                    // 停止所有舵机
                    if (target == "left" || target == "both") {
                        left_servo->stop();
                    }
                    if (target == "right" || target == "both") {
                        right_servo->stop();
                    }
                    
                    ESP_LOGI(TAG, "连续旋转舵机测试完成");
                    delete data;
                    vTaskDelete(nullptr);
                }, "continuous_rotation_test", 4096, new std::tuple<Servo360*, Servo360*, std::string, std::string>(left_servo_, right_servo_, target, test_type), 5, nullptr);
                
                return true;
            });
    }
    
    ~ServoMcpController() {
        // 清理舵机对象
        if (left_servo_) {
            delete left_servo_;
            left_servo_ = nullptr;
        }
        if (right_servo_) {
            delete right_servo_;
            right_servo_ = nullptr;
        }
        
        // 清理 MCPWM 资源
        if (shared_timer_) {
            mcpwm_timer_disable(shared_timer_);
            mcpwm_del_timer(shared_timer_);
            shared_timer_ = nullptr;
        }
    }
    
private:
    mcpwm_timer_handle_t shared_timer_;
    Servo360* left_servo_;
    Servo360* right_servo_;
    bool initialized_;
};

// 在 board 初始化或 main 中实例化
// ServoMcpController servo_ctrl(GPIO_NUM_18, GPIO_NUM_17);