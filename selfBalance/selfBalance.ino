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
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

/* Variables needed for PID controller */
double convertVal = 0.001;
double Input, Output, Setpoint;
double Kp=30, Ki=100, Kd=0;

/* Initializes PID function with above variables */
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/* Variables to help with PID function */
bool flip = 0;
bool store = 0;
bool flipdirection = 0;

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

    /* Calibration value obtained from testing */
    mpu.setYAccelOffset(-1243);
}

void loop()
{   
    /* Stores data from IMU */
    /*
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    Tmp=Wire.read()<<8|Wire.read();
    GyX=Wire.read()<<8|Wire.read();
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read(); 
    */

    /* Library utilized to read MPU data */
    AcY = mpu.getAccelerationY(); 

    //Serial.println(AcY); 

    /* Measures which way robot is leaning */
    if (AcY > 0)
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
    Input = -abs(convertVal * AcY);
    //Input = convertVal * AcY;

    /* Computes PID output to change motor speed */
    myPID.Compute(flip);

    /* Determines direction of motors and inputs to motor function */
    if (AcY < 0)
    {
      driveControl(1, Output);
      //Serial.println(convertVal * AcY);
      Serial.println(Output); 
    }
    else if (AcY > 0)
    {
      driveControl(1, -Output);
      //Serial.println(convertVal * AcY);
      Serial.println(-Output); 
    }
    else
    {
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
      //Serial.print("0");
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
      //Serial.print("1");
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

    //Serial.println("Recieved");
    
    //Serial.print("Motor ");
    //Serial.print(motnum);
    //Serial.print(" at power: ");
    //Serial.println(abs(input));
}
