#include <Wire.h>       // Wire library - used for I2C communication
 int ADXL345 = 0x53;       // The ADXL345 sensor I2C address
 float X_out, Y_out, Z_out;  
 void setup() 
 {
   pinMode(LED_BUILTIN, OUTPUT);
   Serial.begin(9600); 
   Wire.begin(); 
   // Set ADXL345 in measuring mode
   Wire.beginTransmission(ADXL345);  // Start communicating with the device 
   Wire.write(0x2D);                 // Access/ talk to POWER_CTL Register - 0x2D
   // Enable measurement
   Wire.write(8);                    // (8dec -> 0000 1000 binary) 
   Wire.endTransmission();
   delay(10);
 }

float ADXL()
{
  Wire.beginTransmission(ADXL345);  // Start communicating with the device 
   Wire.write(0x2D);                 // Access/ talk to POWER_CTL Register - 0x2D
   // Enable measurement
   Wire.write(8);                    // (8dec -> 0000 1000 binary) 
   Wire.endTransmission();
   delay(10);
   //
   // Read acceleromter data 
   Wire.beginTransmission(ADXL345);
   Wire.write(0x32);                   // Start with register 0x32 (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom(ADXL345, 6, true); // Read 6 registers, each axis in 2 registers
   X_out = ( Wire.read()| Wire.read() << 8); // X-axis value
   X_out = X_out/256;          //For a range of +-2g, divide the raw values by 256     
   Y_out = ( Wire.read()| Wire.read() << 8); // Y-axis value
   Y_out = Y_out/256;
   Z_out = ( Wire.read()| Wire.read() << 8); // Z-axis value
   Z_out = Z_out/256;
 float w= atan(Y_out/X_out);
 return w;
}
float sum_err;
float prev_et;
float Kp=1;
float Ki=1;
float Kd=1;
float dt=1;
float PID_Motor(float error)
  {
    sum_err += error * dt;
    float P = Kp * error;
    float I = Ki * sum_err;
    float D = Kd * (error - prev_et)/dt;
    float ut = P+D;
    prev_et=error;
    return ut;
  }
 float error;
 void loop() 
 {
  float w1 = ADXL();
  delay(10);
  float w2 = ADXL();
  error = 90-(w2-w1);
  PID_Motor( error);
  
 }
 
