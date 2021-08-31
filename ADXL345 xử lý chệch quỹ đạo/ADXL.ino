//re phai x giam re trai x tang

#include <Wire.h>  // Wire library - used for I2C communication
int ADXL345 = 0x53; // The ADXL345 sensor I2C address
float X_out, Y_out, Z_out;
float pre_e1 = 0,pre_e = 0;// Outputs
int h =1;
float post;
float Kp = 1,Ki = 0.02,Kd = 0.03;
void setup() {
  Serial.begin(9600); // Initiate serial communication for printing the results on the Serial monitor
  Wire.begin(); // Initiate the Wire library
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device 
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable 
  Wire.endTransmission();
  delay(10);
}
float locnhieu(float x,int n)
{
  float a[10];
  float sum = 0;
  int i;
  for(i = 0;i<n;i++)
  {
    a[i] = x;
    sum+= a[i];
  }
  return (sum/n);
  
}
float pd(float e,float e_truoc)
{
  float dt = 0.1;
  e_truoc = 0;
  float p = Kp*e;
  float d = Kd*(e-e_truoc)/dt;
  return p+d;
  e_truoc = e;
}
float pid(float e,float sum_e,float pre_e)
{
  float dt = 0.1;
   sum_e += e*dt;
  float p = Kp *e;
  float i = Ki *sum_e;
  float d = (e - pre_e)/dt*Kd*0;
  return p+i;
  pre_e = e;
}
float laygt(float k)
{
  float X[100],q=0;
  int i = 1;
  while(i<=100)
  {
    X[i] = k;
    i++;
  }
  delay(100);
  for(int i =1;i<=100;i++)
  {
    q+=X[i];
  }
  return q/100;
}
//void quayphai(float X_out,int degree,float post) //khi quay 90 do sang phai thi gia tri x se giam 25, quay sang trai thi gia tri x tang 25.
//{
//float a = post - 25;
//Serial.print("tin hieu quay phai: ");
//Serial.println(pd(abs(X_out*100-a),pre_e1));
//}
void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = ( Wire.read()| Wire.read() << 8); // X-axis value
  X_out = X_out/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  if(h>0)
  {
      Serial.print("Gia tri dc lay la :");
      Serial.println(laygt(X_out*100));
      post = laygt(X_out*100);
      h--;
  }

  Serial.print("Xa= ");
  Serial.println(X_out*100);
//  Serial.print("tin hieu ra: ");
//  Serial.println(pd(X_out*100-post,pre_e));
Serial.print("tin hieu quay phai la :");
Serial.println(pd(post+25-X_out*100,pre_e));
 delay(800);
}