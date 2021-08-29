#include<Servo.h>
Servo ServoPen;
Servo ServoClean1;
Servo ServoClean2;
long previousTimes = millis();
int gocquay;
void setup() {
  ServoPen.attach(19);
  ServoClean1.attach(51);
  ServoClean2.attach(53);
}
void StopServo() {
  ServoPen.write(0);
  ServoClean1.write(0);
  ServoClean2.write(0);
}
void Draw (int delay_time_draw) 
{
  long currentTimes = millis();
  
  gocquay = (currentTimes - previousTimes >= delay_time_draw) ? 0 : 180; 
  ServoPen.write(gocquay);
  
  
}
void Clean (int delay_time_clean) {
  long currentTimes = millis();
  gocquay = (currentTimes - previousTimes >= delay_time_clean) ? 0 : 180 ;
    ServoClean1.write(gocquay);
    ServoClean2.write(gocquay);
}
void loop() {
  
//  StopServo();
  Draw(5000);
  delay(1000);
  
//  Clean(5000);
  // put your main code here, to run repeatedly:

}
