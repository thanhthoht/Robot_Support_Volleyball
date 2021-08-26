#include <Servo.h>

#include <LiquidCrystal_I2C.h>

#define motor3 7
#define motor4 8
#define motor1 13
#define motor2 12
#define en1 11
#define en2 9
#define TRIG 20
#define ECHO 21
#define TIME_OUT 5000

int SA=5;
int SB=4;
long counter_1=0;
long c2=0;
int M1=HIGH,M2=LOW;
Servo ServoPen;
Servo ServoClean1;
Servo ServoClean2;
long previousTimes = millis();
int gocquay;
LiquidCrystal_I2C lcd(0x3F,44,2);

void setup()
{

  pinMode(SA,INPUT);
  attachInterrupt(1,readEncoder_L,RISING);
  pinMode(SB,INPUT);
  attachInterrupt(0,readEncoder_R,RISING);
  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  pinMode(motor3,OUTPUT);
  pinMode(motor4,OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);
  ServoPen.attach(10);
  ServoClean1.attach(51);
  ServoClean2.attach(53);
  lcd.begin();
}

int valor1;
void readEncoder_L(){
  valor1=digitalRead(SA);
  if(valor1==1){
    counter_1++;
  }
  else{
    counter_1--;
  }
}
int valor2;

void readEncoder_R(){
  valor2=digitalRead(SB);
  if(valor2==0){
    c2++;
  }
  else{
    c2--;
  }
}
//                            Tính số vòng quay của bánh
//                            Đây là bài toán trên thực tế nên không đưa vào proteus
/*
float encoder_1(){
  static uint32_t previousMillis;
  if (millis() - previousMillis >= 1000) {
            rpm_1 = (counter_1/20)*60;          
            counter_1 = 0;
            previousMillis += 1000;
  }
  return rpm_1;
}

float encoder_2(){
  static uint32_t previousMillis;
  if (millis() - previousMillis >= 1000) {
            rpm_2 = (counter_2/20)*60;          
            counter_2 = 0;
            previousMillis += 1000;
  }
  return rpm_2;
}

*/
// Điều khiển Servo

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


// Điều khiển motor
void motor_up(int in1,int in2,int en,int speed){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  analogWrite(en,speed);
}
void motor_back(int in1,int in2,int en,int speed){
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  analogWrite(en,speed);
}
void stop(){
  digitalWrite(motor1,LOW);
  digitalWrite(motor2,LOW);
  digitalWrite(motor3,LOW);
  digitalWrite(motor4,LOW);
}
void turn_Left(int speed){        // Motor 1-2 lùi, Motor 3-4 tiến
  motor_back(motor1,motor2,en1,speed);
  motor_up(motor3,motor4,en2,speed);
}
void turn_Right(int speed){       // Motor 1-2 tiến, Motor 3-4 lùi
  motor_back(motor3,motor4,en2,speed);
  motor_up(motor1,motor2,en1,speed);
}

//                                Thiết lập bộ điều khiển PID cho Motor
float Kp = 1;
float Ki = 0.5;
float Kd = 0.05;
float dt = 10;
float sum_err=0;
float prev_et=0;
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
            
int control_Motor_By_PID_up(float error){
  float velocity=104+PID_Motor(error);  
  // vận tốc gốc để Motor encoder quay trong proteus là 100
  motor_up(motor1,motor2,en1,velocity);                                  
  motor_up(motor3,motor4,en2,velocity);
  return velocity;
}

int control_Motor_By_PID_TurningRight_U(float error){
  float velocity=104+PID_Motor(error);  
  if(get_distance==20){
    velocity=0;
  }
  motor_up(motor1,motor2,en1,velocity);                                
  return velocity;
}

int control_Motor_By_PID_TurningLeft_U(float error){
  float velocity=104+PID_Motor(error);  
  motor_up(motor3,motor4,en2,velocity);                                  
  return velocity;
}

//                              Thiết lập bộ điều khiển PID cho góc quay


float Kp_a = 1;
float Ki_a = 0.5;
float Kd_a = 0.5;
float dt_a = 0.01;
float sum_err_a=0;
float prev_et_a=0;
float PID_Angle(float error)
{
  sum_err_a += error * dt_a;
  float P = Kp_a * error;
  float I = Ki_a * sum_err_a;
  float D = Kd_a * (error - prev_et_a)/dt_a;
  float ut = P + I + D;
  prev_et_a=error;
  return ut;
}
//                        Cảm biến siêu âm

float prev_distanceCm = 0;
float get_distance()
{
  long duration;
  float distanceCm;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  duration = pulseIn(ECHO, HIGH, TIME_OUT);
 
  // convert to distance
  distanceCm = (float)duration / 29.1 / 2;
  if(distanceCm == 0)
  {
    if (prev_distanceCm == 0) prev_distanceCm = 20;
    distanceCm = prev_distanceCm;
  } 
  prev_distanceCm = distanceCm;
  return distanceCm;
}

void Delay(float delaytime,void(func)()){
  unsigned long endTime=millis()+delaytime;
  while(millis()<endTime){
    func();
    while(millis()<endTime){};
  }
}

int setpoint_Distance_1=100;
int setpoint_Angle_1=90;
int setpoint_Distance_2=60;
int setpoint_Stop_Barrier=20;
int counter_2=0;
float error1;   // Quãng đường
float error2;   // Góc quay
float distance;   // Dừng trước vật cản

const int historyLength = 10;
float history[historyLength];
int historyIndex=0;


int getStableValue() {
  long sum = 0;
  for (int l = 0; l < historyLength; l++) {
    sum += history[l];
  }
  return sum / historyLength;
}


void loop()
{
  while(counter_1<=100){
    error1=setpoint_Distance_1-counter_1;
    int l=control_Motor_By_PID_up(error1);
    
    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);
  }

  Delay(500,stop);
    lcd.clear();
  counter_1=0;
  while(counter_1<=50){
    error1=setpoint_Distance_1-counter_1;
    int l=control_Motor_By_PID_up(error1);
    
    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);

  }
    Delay(500,stop);
    lcd.clear();
  counter_1=0;

  // int q++;
  // q=2(stop)
}



/*
  //for(int j=0;j<historyLength;j++){
  //    history[j]=100;}
  
    while(counter_1<=100){
    error1=setpoint_Distance_1-counter_1;
    int l=control_Motor_By_PID_up(error1);

    // history[historyIndex]=error1;
    // historyIndex=(historyIndex+1)%historyLength;

    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);
    // if(getStableValue()==0){
    //   break;
    // }
  }
  Delay(1000,stop);
  while(counter_2<=90){
   turn_Right(200);
   lcd.setCursor(0,2);
   lcd.print(counter_2);
   counter_2++;
  }
  counter_1=0;
  counter_2=0;
  lcd.clear();
  Delay(1000,stop);
*/