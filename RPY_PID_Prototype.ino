// Uses jarzebski's MPU6050 libarary, seems to be the simplest implementation of the functions
// that we need, github link is https://github.com/jarzebski/Arduino-MPU6050

#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
Servo test_servo;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

// Servo angle initializer
float angle = 90;
float targetAngle = -5;
float errorProportional = 0.0;
float errorIntegral = 0.0;
float errorDerivative = 0.0;
float errorPrevious = 0.0;
float currentTime = 0;
float previousTime = 0;
float elapsedTime = 0;
float inputServo = 0;

float Kp = 4;
float Ki = 1
float Kd = 0.1;

void setup() 
{
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrating gyroscope, comment if not needed
  mpu.calibrateGyro();

  // Connecting the test servo to pin 9
  test_servo.attach(9);
}

void loop()
{
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);
  Serial.print('\n');

  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  errorPrevious = errorProportional;
  errorProportional = targetAngle - pitch;
  errorIntegral = errorIntegral + errorProportional * elapsedTime;
  errorDerivative = (errorProportional - errorPrevious)/elapsedTime;
  previousTime = currentTime;

  inputServo = Kp * errorProportional + Ki * errorIntegral + Kp * errorDerivative;

  // Simply writting pitch angle to the servo as a test case
  test_servo.write(inputServo+90);

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}
