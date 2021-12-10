/* Imports libraries needed */
#include<Wire.h> 
#include "PID.h" 
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>

MPU6050 mpu;

/* Defines pins for motors */
const int MOT_0_1 = 6;
const int MOT_0_2 = 7;
const int MOT_0_PWM = 10;

const int MOT_1_1 = 3;
const int MOT_1_2 = 4;
const int MOT_1_PWM = 9;
 
const int STANDBY = 5;

/*Sets an initial value for motor speed */
int motSpeed = 0;

/* Variables neeeded for IMU */
const int MPU=0x68;
int16_t ax, ay, az, gx, gy, gz;

/* Bias variables */
float accBiasX, accBiasY, accBiasZ;
float gyroBiasX, gyroBiasY, gyroBiasZ;

/* Variables to store MPU data */
float accAngleX, accAngleY;
double accPitch, accRoll;

float gyroRateX, gyroRateY, gyroRateZ;
float gyroBias_oldX, gyroBias_oldY, gyroBias_oldZ;
float gyroPitch = 180;
float gyroRoll = -180;
float gyroYaw = 0;

/* Variables needed for PID controller */
double convertVal = 0.001;
double Input, Output, Setpoint;
double Kp=16, Ki=3, Kd=.2;
// 16, 0, 0

/* Initializes PID function with above variables */
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/* Variables to help with PID function */
bool flip = 0;
bool store = 0;
bool flipdirection = 0;

/* timer */
uint32_t timer;

/* Complimentary filter variable */
double filtered = 0;

/* Initial setup */
void setup()
{
    /* Initializes IMU */
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    /* For serial communication with computer */
    Serial.begin(9600);

    /* Defines pins for motors */
    pinMode(MOT_0_1, OUTPUT);
    pinMode(MOT_0_2, OUTPUT);
    pinMode(MOT_0_PWM, OUTPUT);

    pinMode(MOT_1_1, OUTPUT);
    pinMode(MOT_1_2, OUTPUT);
    pinMode(MOT_1_PWM, OUTPUT);

    pinMode(STANDBY, OUTPUT);
    digitalWrite(STANDBY, HIGH);

    /* State that PID controller is trying to reach (angle of zero) */
    Setpoint = 0;

    /* Sets PID controller to automatic (computes each time) */
    myPID.SetMode(AUTOMATIC);

    /* Calibration values obtained from testing */
    mpu.setXAccelOffset(-2072);
    mpu.setYAccelOffset(-1287);
    mpu.setZAccelOffset(965);
    mpu.setXGyroOffset(-60);
    mpu.setYGyroOffset(-501);
    mpu.setZGyroOffset(-537);
  
    gyroBiasX = 0;
    gyroBiasY = 0;
    gyroBiasZ = 1;
  
    accBiasX = 6;
    accBiasY = 21;
    accBiasZ = 16381;

    /* Pulls MPU data */
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /* Calculates initial values for Complimemtary filter */
    accPitch = (atan2(-ax, -az) + PI) * RAD_TO_DEG;
    accRoll = (atan2(ay, -az) + PI) * RAD_TO_DEG;

    if (accPitch <= 360 & accPitch >= 180) {
      accPitch = accPitch - 360;
    }

    if (accRoll <= 360 & accRoll >= 180) {
      accRoll = accRoll - 360;
    }

    gyroPitch = accPitch;
    gyroRoll = accRoll;

    timer = micros();
    delay(1000);
}

void loop()
{   
    /* Library utilized to read MPU data */
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /***********************
     * Complimentary filter
     ***********************/
    filtered = complimentaryFilter(ay, az, gx);

    /* Measures which way robot is leaning */
    if (filtered > 0)
    {
      flipdirection = 0;
    }
    else
    {
      flipdirection = 1;
    }

    /* If direction of robot leaning changes, reset the integrator */
    if (flipdirection != store)
    {
      store = flipdirection;
      flip = 1;
    }
    else
    {
      flip = 0;
    }

    /* Changes error to a value PID controller can work with */
    Input = -abs(filtered);

    /*****************
     * PID Controller
     *****************/
    myPID.Compute(flip);

    /* Determines direction of motors and inputs to motor function */
    if (filtered > 0)
    {
      driveControl(1, Output);
      Serial.print(millis());
      Serial.print(';'); 
      Serial.print(filtered);
      Serial.print(';');
      Serial.println(Output); 
      delay(.05);
      driveControl(0, Output);
    }
    else
    {
      driveControl(1, -Output);
      Serial.print(millis());
      Serial.print(';'); 
      Serial.print(filtered);
      Serial.print(';');
      Serial.println(-Output);
      delay(.05);
      driveControl(0, Output); 
    }
    
    //driveControl(1,Output);
    //Serial.println(Output); 
    
    
} // end of loop

/* driveControl(...) **************************************************************
 *     This function is to control the motors in coordination. It is expandable to
 *     work with more than 2 motors and as many different combination of directions
 *     as desired. In this case, it only needs to flip the motor directions relative
 *     to whcih direction the balanicng robot is leaning.
 **********************************************************************************/
void driveControl(int mode, int motSpeed)
{ 
  /* Motors stopped */
  if (mode == 0)
  {
    motorControl(0,0);
    motorControl(1,0);
  }
  /* Motors driving */
  else if (mode == 1) //move
  {
    motorControl(0,-motSpeed);
    motorControl(1,motSpeed);
  }
}

/* motorControl(...) **************************************************************
 *     This function is called per motor and individually controls each motor's
 *     speed. This function can also be easily expanded to control more motors
 *     in conjunction with driveControl(...)
 **********************************************************************************/
void motorControl(int motnum, int input)
{
    /* Left motor control */
    if (motnum == 0)
    {
      analogWrite(MOT_0_PWM, abs(input));
      if (input >= 0)
      {
        digitalWrite(MOT_0_1, HIGH);
        digitalWrite(MOT_0_2, LOW);
      }
      else
      {
        digitalWrite(MOT_0_1, LOW);
        digitalWrite(MOT_0_2, HIGH);
      }
    }
    /* Right motor control */
    else if (motnum == 1)
    {
      analogWrite(MOT_1_PWM, abs(input));
      if (input >= 0)
      {
        digitalWrite(MOT_1_1, HIGH);
        digitalWrite(MOT_1_2, LOW);
      }
      else
      {
        digitalWrite(MOT_1_1, LOW);
        digitalWrite(MOT_1_2, HIGH);
      }
    }
    else
    {
      //nothing
    }
}

/* complimentaryFilter(...) *******************************************************
 *     This is a complimentary filter, a computationally inexpensive sensor fusion 
 *     technique that consists of a low-pass and a high-pass filter. In this use,
 *     it allows the MPU data to be filtered and avoid noise and overcorrection
 *     by th PID controller.
 **********************************************************************************/
double complimentaryFilter(int16_t ay, int16_t az, int16_t gx){
    accRoll = (atan2(ay/182.0, -az/182.0) + PI) * RAD_TO_DEG;

    if (accRoll <= 360 & accRoll >= 180) {
      accRoll = accRoll - 360;
    }

    gyroRateX = -((int)gx - gyroBiasX) / 131; 

    double gyroVal = gyroRateX * ((double)(micros() - timer) / 1000000);

    timer = micros();

    /* Complementary filter value */
    return 0.98 * (filtered + gyroVal) + 0.02 * (accRoll);
}