
#include "PID.h"

float const Kp = 25.0;
float const Ki = 250;
float const Kd = 1.01;

float PID_Compute(PIDController *pid, float setpoint, float measured_value, float delta_t) {
  // Tính sai số
  float error = setpoint - measured_value;

  // Thành phần tỉ lệ
  float P = pid->Kp * error;

  // Thành phần tích phân
  pid->integral += error * delta_t;
//  if ( pid->integral > 100) {
//    pid->integral = 100;
//  }
//
//  if ( pid->integral < -100) {
//    pid->integral = -100;
//  }

  float I = pid->Ki * pid->integral;
  //  if(I> 30){
  //    i = 30;
  //  }
  // Thành phần vi phân
  float derivative = (error - pid->prev_error) / delta_t;
  float D = pid->Kd * derivative;

  // Cập nhật sai số trước
  pid->prev_error = error;
//    Serial.print(P);
//   Serial.print(", ");
//   Serial.print(I);
//   Serial.print(", ");

//   Serial.print(D);
//   Serial.print(",         ");
  
  // Tín hiệu điều khiển
  return P + I + D;
}