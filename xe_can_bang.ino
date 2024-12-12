#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define TEST_MOTOR 0

typedef struct PIDController {
  float Kp;        // Hệ số tỉ lệ
  float Ki;        // Hệ số tích phân
  float Kd;        // Hệ số vi phân
  float prev_error; // Sai số ở chu kỳ trước
  float integral;   // Tích phân của sai số
} PIDController;


MPU6050 mpu;
int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.
#define OUTPUT_READABLE_YAWPITCHROLL
/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
/*---Orientation/Motion Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
/*-Packet structure for InvenSense teapot demo-*/
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}





int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int minZ = 0;
int maxZ = 0;
int offX = 0;
int offY = 0;
int offZ = 0;


#define IN1  7
#define IN2 6
#define IN3 12
#define IN4 11
#define LS 10
#define RS 9
#define MAX_SPEED 255 //từ 0-255
#define MIN_SPEED 0


void left_tien(uint8_t sp) {

  if (sp <= MIN_SPEED) sp = MIN_SPEED;
  if (sp >= MAX_SPEED) sp = MAX_SPEED;
  digitalWrite(IN1, HIGH);// chân này không có PWM
  digitalWrite(IN2, LOW);// chân này không có PWM
//  Serial.print("left tien ");
//  Serial.println(sp);
  analogWrite(LS, sp);
}


void left_lui(uint8_t sp) {
  if (sp <= MIN_SPEED) sp = MIN_SPEED;
  if (sp >= MAX_SPEED) sp = MAX_SPEED;
  digitalWrite(IN1, LOW);// chân này không có PWM
  digitalWrite(IN2, HIGH);// chân này không có PWM
//  Serial.print("left lui ");
//  Serial.println(sp);
  analogWrite(LS, sp);
}


void right_tien(uint8_t sp) {
  if (sp <= MIN_SPEED) sp = MIN_SPEED;
  if (sp >= MAX_SPEED) sp = MAX_SPEED;
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);// chân này không có PWM
//  Serial.print("right tien ");
//  Serial.println(sp);
  analogWrite(RS, sp);
}


void right_lui(uint8_t sp) {
  if (sp <= MIN_SPEED) sp = MIN_SPEED;
  if (sp >= MAX_SPEED) sp = MAX_SPEED;
  digitalWrite(IN4, LOW);// chân này không có PWM
  digitalWrite(IN3, HIGH);// chân này không có PWM
//  Serial.print("right lui ");
//  Serial.println(sp);
  analogWrite(RS, sp);
}

void left_dung() {
  digitalWrite(IN1, LOW);
  digitalWrite(LS, LOW);
//  Serial.println("left dung");
  digitalWrite(IN2, LOW);
}

void right_dung() {
  digitalWrite(IN3, LOW);
  digitalWrite(RS, LOW);
//  Serial.println("right dung");
  digitalWrite(IN4, LOW);
}


void init_sensor() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  //  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }


  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //    Serial.println(F(")..."));
    //    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  }
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
}

void sensor_read() {
  if (!DMPReady) return; // Stop the program if DMP programming fails.

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //      Serial.print("ypr\t");
    //      Serial.print(ypr[0] * 180/M_PI); // quay mt ngang
    //      Serial.print("\t");
    //          Serial.println(ypr[1] * 180/M_PI); // quay truoc sau (nghieeng )
    //      Serial.print("\t");
    //      Serial.println(ypr[2] * 180/M_PI); // quay 2 been
#endif

  }

}

int left = 0;
int right = 0;

uint32_t stamp = 0;
float dt = 0.0;
float err = 0;
float preError = 0;

#define thread_Hold 30.0
float const Kp = 25.0;
float const Ki = 250;
float const Kd = 1.01;
float const TARGET = 0.0;
PIDController PID;

int base = 30;

int stateMachine = 0;

float result = 0;

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->prev_error = 0.0f;
  pid->integral = 0.0f;
  return;
}



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
 Serial.print(P);
  Serial.print(", ");
  Serial.print(I);
  Serial.print(", ");

  Serial.print(D);
  Serial.print(",         ");
  
  // Tín hiệu điều khiển
  return P + I + D;
}


void controlMotor(float res) {
//  res = res > 255.0 ? 255.0 : res;
//  res = res < -255.0 ? -255.0 : res;
//  Serial.print("process res ");
//  Serial.println(res);
  int ret = (int)res;
  int r = right + base; 
  int l = left + base ; 
  
//    res = res > 255.0 ? 255.0 : res;
////  res = res < -255.0 ? -255.0 : res;
  if (ret == 0) {
    left_dung();
    right_dung();
    return ;
  }

  
  if (ret > 0) {
   r += ret; 
     r = (r)>255?255: (r); 
    r = (r)<-255?-255:(r); 
    l =l  + ret; 
     l =  (l)>255?255: (l); 
    l = (l)<-255?-255:(l); 
    right_tien(r);
    left_tien(l);
    return;
  }

  if (ret < 0) {
   r =r - ret; 
     r = (r)>255?255: (r); 
    r = (r)<-255?-255:(r); 
  l =l - ret; 
     l =  (l)>255?255: (l); 
    l = (l)<-255?-255:(l); 
    right_lui(r);
    left_lui(l);
    return;
  }

}
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LS, OUTPUT);
  pinMode(RS, OUTPUT);
  Serial.begin(115200);
#if(TEST_MOTOR)
  uint8_t LV[] = {0, 60, 100, 120, 128, 156, 255};
  uint8_t level = 0;
  uint8_t mode = 0;
  while (1) {
    while (!Serial.available());
    char c =  Serial.read();
    switch (c) {
      case 'l':
        if (mode == 0) {
          left_tien(LV[level]);
        } else {
          left_lui(LV[level]);

        }

        break;

      case 'r':
        if (mode == 0) {
          right_tien(LV[level]);
        } else {
          right_lui(LV[level]);

        }
        break;

      case 's':
        left_dung();
        right_dung();
        break;

      case 'm':
        Serial.println("change mode");
        mode += 1;
        mode = mode % 2;
        break;

      case 't' : //
        level += 1;
        level %= sizeof(LV);
        break;

      default:
        break;
    }
  }
#endif
  //  Serial.println("Starting the I2C interface.");
  init_sensor();
  PID_Init(&PID, Kp, Ki, Kd);
  // Serial.println("tien");
  // right_tien(right + base);
  //    left_tien(left + base );
  //   delay(2000);
  //   Serial.println("Lui");
  //     right_lui(right + base  );
  //    left_lui(left +base );
  //  delay(2000);
  //  Serial.println("dung");
  //     left_dung();
  //    right_dung();

  //    while(1);
}

void loop() {

  switch (stateMachine) {
    case 0:// prepare;
      if (millis()  - stamp > 1000) {
        stateMachine = 1;
        PID_Init(&PID, Kp, Ki, Kd);
        dt = 0;
        stamp = millis();
      }
      break;
    case 1:
      // read data
      sensor_read();
      dt = (float)(millis() - stamp) / 1000.0;
      stamp = millis();
      err = ypr[1] * 180 / M_PI;
      if (err < -thread_Hold || err > thread_Hold) {
        stateMachine = 2;
        break;
      }
      result = PID_Compute(&PID, TARGET, err , dt);
      controlMotor(result);
      Serial.print(err);
      Serial.print(",");
      Serial.print(result);
      Serial.print(",");
      Serial.println(base + abs(result));
      break;
    case 2: // stop
      left_dung();
      right_dung();
      stateMachine = 0;
      break;

    default:
      stateMachine = 0;
      break;
  }
}
