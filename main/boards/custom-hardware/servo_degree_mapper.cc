#include "servo_degree_mapper.h"
#include <iostream>
#include <cmath>

int LookupDuration(int degree, const ServoCalibration& calibration) {
    int abs_degree = std::abs(degree);
    // 优先查表
    auto it = calibration.degree_to_duration.find(abs_degree);
    if (it != calibration.degree_to_duration.end()) {
        return it->second;
    }
    // 无查表项则用公式
    return abs_degree * calibration.full_circle_time_ms / 360;
}

void DegreeToServoParams(int degree, const ServoCalibration& calibration, int& out_speed, int& out_duration) {
    int abs_degree = std::abs(degree);
    int direction = (degree >= 0) ? 1 : -1;
    out_speed = direction * calibration.speed;
    out_duration = LookupDuration(degree, calibration);
}

void SendServoAction(const std::string& target, int speed, int duration) {
    // 实际项目中请替换为MCP协议调用
    std::cout << "MCP协议下发: target=" << target << " speed=" << speed << " duration=" << duration << std::endl;
    // 例如：
    // mcp_server.call("self.servo360.set", {{"target", target}, {"speed", speed}, {"duration", duration}});
}

void HandleRaiseArm(const std::string& target, int degree, const ServoCalibration& calibration) {
    int speed, duration;
    DegreeToServoParams(degree, calibration, speed, duration);
    SendServoAction(target, speed, duration);
} 