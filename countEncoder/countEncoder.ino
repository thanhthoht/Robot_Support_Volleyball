#include<Servo.h>
Servo ServoPen;
Servo ServoClean1;
Servo ServoClean2;
long previousTimes = millis();
int gocquay;
void setup() {
  ServoPen.attach(10);
  ServoClean1.attach(51);
  ServoClean2.attach(53);
}
void StopServo() {
  ServoPen.write(0);
  ServoClean1.write(0);
  ServoClean2.write(0);
}
void Draw(int delay_time_draw) 
{
  ServoPen.write(180);
  long currentTimes = millis();
  if (currentTimes - previousTimes >= delay_time_draw) {
    previousTimes = currentTimes;
    ServoPen.write(0);
  }
}
void Clean (int delay_time_clean) {
  ServoClean1.write(180);
  ServoClean2.write(180);
  long currentTimes = millis();
  if (currentTimes - previousTimes >= delay_time_clean) {
    previousTimes = currentTimes;
    ServoClean1.write(0);
    ServoClean2.write(0);
  }
}
void loop() {
  
  StopServo();
  for(int i=0;i<90;i++){
  	ServoPen.write(i++);
  	delay(10);
  }
  // put your main code here, to run repeatedly:

}
