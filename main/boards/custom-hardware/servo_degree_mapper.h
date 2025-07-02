#pragma once
#include <string>
#include <map>

// 舵机标定数据结构
struct ServoCalibration {
    int speed;                // 标定用的速度
    int full_circle_time_ms;  // 该速度下转360°所需时间（ms）
    std::map<int, int> degree_to_duration; // 角度-时间查表（可选）
};

// 角度到舵机协议参数换算
// degree: 目标角度（正负表示方向）
// calibration: 标定数据
// out_speed, out_duration: 输出参数
void DegreeToServoParams(int degree, const ServoCalibration& calibration, int& out_speed, int& out_duration);

// 查表法换算（优先查表，无则用公式）
int LookupDuration(int degree, const ServoCalibration& calibration);

// 协议下发接口声明（实际实现可根据项目需要调整）
void SendServoAction(const std::string& target, int speed, int duration);

// 语义动作处理示例
void HandleRaiseArm(const std::string& target, int degree, const ServoCalibration& calibration);

// 角度指令解析层调用声明
void HandleArmDegreeCommand(const std::string& target, int degree); 