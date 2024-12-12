#ifndef PID_H

extern float const Kp;
extern float const Ki;
extern float const Kd;


typedef struct PIDController {
  float Kp;        // Hệ số tỉ lệ
  float Ki;        // Hệ số tích phân
  float Kd;        // Hệ số vi phân
  float prev_error; // Sai số ở chu kỳ trước
  float integral;   // Tích phân của sai số
} PIDController;

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd); 

float PID_Compute(PIDController *pid, float setpoint, float measured_value, float delta_t); 

#endif