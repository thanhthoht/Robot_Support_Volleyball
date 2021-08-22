int IN1 = 9;
int IN2 = 8;
int IN3 = 7;
int IN4 = 6;
int enA=10;
int enB=5;
int Encoder_1=2;   // Phai
int Encoder_2=3;  //Trai
int trig = 12;
int echo = 13;
int state;


volatile unsigned int counter_1;
int rpm_1;
float sum_err_1=0;
float prev_et_1=0;

volatile unsigned int counter_2;
int rpm_2;
float sum_err_2=0;
float prev_et_2=0;

  
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);


// Encoder phải
  pinMode(Encoder_1,INPUT);
  digitalWrite(Encoder_1, HIGH);
  attachInterrupt(0, countpulse_1, RISING); 
// Encoder trái  
 pinMode(Encoder_2,INPUT);
 digitalWrite(Encoder_2, HIGH);
 attachInterrupt(1, countpulse_2, RISING); 

  Serial.begin(9600);
}

void countpulse_1(){
        counter_1++;
}

void countpulse_2(){
        counter_2++;
}

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

// ĐO KHOẢNG CÁCH ĐẾN VẬT
float Khoang_cach(){
  float duration;               // biến lưu thời gian
  float distance;               // biến lưu khoảng cách
  digitalWrite(trig,LOW);       // tắt trig
  delayMicroseconds(2);
  digitalWrite(trig,HIGH); 
  delayMicroseconds(10);
  digitalWrite(trig,LOW); 
  duration = pulseIn(echo,HIGH,30000);
  distance = (duration*0.034/2);
  return distance;
}

// ĐIỀU KHIỂN 2 ĐỘNG CƠ
void Motor_1(int v){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(enA,v); 
}

void Motor_2(int v){
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(enB,v);
}

void PID_1(float et)
{
  int Kp = 10;
  int Ki = 0.0001;
  int Kd = 0.02;
  float dt = 0.01;
  
  sum_err_1 += et * dt;
  float P = Kp * et;
  float I = Ki * sum_err_1;
  float D = Kd * (et - prev_et_1)/dt;
  float ut = P + I + D;

    if(int(ut)>255){
      ut=255;
    }
    Motor_1(abs(int(ut)));
    prev_et_1 = et;
  delay(10);
}

void PID_2(float et)
{
  int Kp = 200;
  int Ki = 0;
  int Kd = 0;
  float dt = 0.01;
  
  sum_err_2 += et * dt;
  float P = Kp * et;
  float I = Ki * sum_err_2;
  float D = Kd * (et - prev_et_2)/dt;
  float ut = P + I + D;
    if (sum_err_2 >4000) sum_err_2 = 4000;
    if (sum_err_2 <-4000) sum_err_2 = -4000;
  
  delay(10);
}
void BlueTooth(){
  if(Serial.available()>0){
    state= Serial.read();
    switch (state){
      case '1': // Tien
        Motor_1(255);
        Motor_2(255);
      break;
      case '2': // Lui
        Motor_1(-255);
        Motor_2(-255);
      break;
      case '3': // Phai
        Motor_1(0);
        Motor_2(255);
      break;
      case '4': // Trai
        Motor_1(255);
        Motor_2(0);
      break;
    }
  }
}
void loop() {
BlueTooth();
}