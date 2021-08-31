#include <LiquidCrystal_I2C.h>

#include <Servo.h>

// KHAI BÁO CHÂN
// Chân 2 motor sau
#define motor1 7
#define motor2 6
#define motor3 8
#define motor4 9

//Chân 2 motor trước
#define motor5 12
#define motor6 10
#define motor7 13
#define motor8 11

//Chân cảm biến siêu âm
#define TRIG_1 23
#define ECHO_1 24
#define TRIG_2 25
#define ECHO_2 26

//Chân encoder
#define SA 5    // Encoder trái
#define SB 4    // Encoder phải

// động cơ Servo
Servo ServoPen;
Servo ServoClean1;
Servo ServoClean2;

long counter_1=0;   // đếm xung encoder của chân trái
long counter_2=0;   // đếm xung encoder của chân phải
                    // Ta sẽ giả định đây là quãng đường trong giả lập proteus
int M1=HIGH,M2=LOW;
long previousTimes = millis();
int gocquay;
LiquidCrystal_I2C lcd(0x3F,44,2);


void setup(){
  pinMode(motor1,OUTPUT);   //trái dưới
  pinMode(motor2,OUTPUT);
  pinMode(motor3,OUTPUT);   //phải dưới
  pinMode(motor4,OUTPUT);
  pinMode(motor5,OUTPUT);   //trái trên
  pinMode(motor6,OUTPUT);
  pinMode(motor7,OUTPUT);   //phải trên
  pinMode(motor8,OUTPUT);

  pinMode(SA,INPUT);                        //Encoder trái
  attachInterrupt(1,readEncoder_L,RISING);
  pinMode(SB,INPUT);                        //Encoder phải
  attachInterrupt(0,readEncoder_R,RISING);

  pinMode(TRIG_1,OUTPUT);   // CBSA trước
  pinMode(ECHO_1,INPUT);
  pinMode(TRIG_2,OUTPUT);   // CBSA bên phải để bám tường phải
  pinMode(ECHO_2,INPUT);
  ServoPen.attach(10);
  ServoClean1.attach(51);
  ServoClean2.attach(53);
  lcd.begin();              //Bật LCD
}

            //Đếm xung Encoder

int valor1;
void readEncoder_L(){
  valor1=digitalRead(SA);
  if(valor1==0){
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
    counter_2++;
  }
  else{
    counter_2--;
  }
}

// Các hàm điều khiển trạng thái Servo
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

//      Các hàm trạng thái điều khiển cho động cơ
void motor_up(int speed,int in1,int in2){
    digitalWrite(in1,HIGH);
    analogWrite(in2,255-speed);
}
void motor_back(int speed,int in1,int in2){
    digitalWrite(in1,LOW);
    analogWrite(in2,speed);
}
void stop(){
  digitalWrite(motor1,LOW);
  digitalWrite(motor2,LOW);
  digitalWrite(motor3,LOW);
  digitalWrite(motor4,LOW);
  digitalWrite(motor5,LOW);
  digitalWrite(motor6,LOW);
  digitalWrite(motor7,LOW);
  digitalWrite(motor8,LOW);
}

//           Thiết lập bộ điều khiển PID cho Motor
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
  // if(get_distance==20){
  //   velocity=0;
  // }
  // vận tốc gốc để Motor encoder quay trong proteus là 100
  motor_up(velocity,motor1,motor2);                                  
  motor_up(velocity,motor3,motor4);
  motor_up(velocity,motor5,motor6);
  motor_up(velocity,motor7,motor8);
  return velocity;
}

int control_Motor_By_PID_SpinRight(float error){    //Hàm xoay phải
  float velocity=70+PID_Motor(error);  
  motor_up(velocity,motor1,motor2);
  motor_up(velocity,motor5,motor6);
  motor_back(velocity,motor3,motor4);
  motor_back(velocity,motor7,motor8);                                
  return velocity;
}

int control_Motor_By_PID_SpinLeft(float error){     //Hàm xoay trái
  float velocity=70+PID_Motor(error);  
  motor_back(velocity,motor1,motor2);
  motor_back(velocity,motor5,motor6);
  motor_up(velocity,motor3,motor4);
  motor_up(velocity,motor7,motor8);                               
  return velocity;
}

int control_Motor_By_PID_TurnRight(float error){     //Hàm quay phải để quay đầu dùng trong hàm lau sàn
  float velocity=90+PID_Motor(error);  
  motor_up(velocity,motor1,motor2);         //Động cơ trái dưới quay, 3 động cơ còn lại dừng                         
  return velocity;
}

int control_Motor_By_PID_TurnLeft(float error){     //Hàm quay trái để quay đầu dùng trong hàm lau sàn
  float velocity=90+PID_Motor(error);  
  motor_up(velocity,motor3,motor4);         //Động cơ phải dưới quay, 3 động cơ còn lại dừng                                  
  return velocity;
}

// Hàm Delay, trong lúc Delay sẽ có time để chỉnh Servo
void Delay(float delaytime,void(func)()){
  unsigned long endTime=millis()+delaytime;
  while(millis()<endTime){
    func();
    while(millis()<endTime){};
  }
}

// Đo đạc khoảng cách bằng cảm biến siêu âm và PID bám tường

float Measure_distance(int trig,int echo)
{
    unsigned long duration; 
    float distance;           
    digitalWrite(trig,0);   
    delayMicroseconds(2);
    digitalWrite(trig,1);   
    delayMicroseconds(5);   
    digitalWrite(trig,0);
    duration = pulseIn(echo,HIGH,3000); 
    distance = int(duration/2/29.412);
    return distance;
}
void phathienvat()
{
  if (Measure_distance(TRIG_1,ECHO_1)<=20||Measure_distance(TRIG_2,ECHO_2)) 
  {
    Delay(200,stop);
    SpinLeft90();
    PID_Distance(100);
}
}
float Kp_d = 1;
float Ki_d = 0.5;
float Kd_d = 0.05;
float dt_d = 10;
float sum_err_d=0;
float prev_et_d=0;
float setpoint_d_Right = 20;
void PID_Distance(float error)
{
  error = setpoint_d_Right - Measure_distance(TRIG_2,ECHO_2);
  sum_err += error * dt;
  float P = Kp_d * error;
  float I = Ki_d * sum_err_d;
  float D = Kd_d * (error - prev_et_d)/dt_d;
  float ut = P+I+D;
  prev_et=error;

    digitalWrite (motor1, HIGH);
    analogWrite (motor2, 110);
    digitalWrite (motor3, HIGH);
    analogWrite (motor4, 110+ut);
}


// Setup tham số trước vòng loop
int setpoint_Distance_1=100;    // Quãng đường đi chiều dài thẳng trong sân
int setpoint_Distance_2=60;     // Quãng đường đi chiều rộng thẳng trong sân
int setpoint_Distance_3=32;     // Quãng đường để quay 1 góc 90 độ xx 10Pi=31.41
int setpoint_Distance_4=63;     // Quãng đường để quay đầu 180 độ xx 20Pi=62.83
float error1;   // Tính lỗi quãng đường
float distance;   // Dừng trước vật cản

// Giả định counter_1 sẽ là quãng đường
// Khi áp dụng bài toán dùng ADXL để giữ ổn định thì khi chạy thẳng 2 encoder sẽ đếm đc xung xấp xỉ nhau
void SpinLeft90(){
  counter_2=0;
  while(counter_1<=setpoint_Distance_3){

    error1=setpoint_Distance_1-counter_2;
    int l=control_Motor_By_PID_SpinLeft(error1);
  }
  counter_2=0;
}

void SpinRight90(){
  counter_2=0;
  while(counter_1<=setpoint_Distance_3){

    error1=setpoint_Distance_1-counter_2;
    int l=(error1);
  }
  counter_2=0;
}

void SpinLeft180(){
  counter_1=0;
  while(counter_1<=setpoint_Distance_4){

    error1=setpoint_Distance_1-counter_1;
    int l=control_Motor_By_PID_SpinLeft(error1);
  counter_1=0;
}
}

void SpinRight180(){
  counter_1=0;
  while(counter_1<=setpoint_Distance_4){

    error1=setpoint_Distance_1-counter_1;
    int l=control_Motor_By_PID_SpinLeft(error1);
  counter_1=0;
}
}

void PaintingMap(){
  counter_1=0;
  while(counter_1<=setpoint_Distance_1){

    error1=setpoint_Distance_1-counter_1;
    int l=control_Motor_By_PID_up(error1);
    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);
  }

  Delay(300,stop);
    lcd.clear();
  counter_1=0;

  while(counter_1<=setpoint_Distance_3){

    error1=setpoint_Distance_1-counter_1;
    int l=control_Motor_By_PID_SpinRight(error1);
    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);
    }
    Delay(300,stop);
    lcd.clear();
  counter_1=0;

  while(counter_1<=setpoint_Distance_2){

    error1=setpoint_Distance_2-counter_1;
    int l=control_Motor_By_PID_up(error1);
    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);
  }

  Delay(300,stop);
    lcd.clear();
  counter_1=0;
  while(counter_1<=setpoint_Distance_3){

    error1=setpoint_Distance_1-counter_1;
    int l=control_Motor_By_PID_SpinRight(error1);
    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);
    }
    Delay(300,stop);
    lcd.clear();
  counter_1=0;
}

// Lau sàn + Đi vòng qua vật cản có điều kiện

int status=0;       // biến chỉnh trạng thái
int prev_counter=0;       // Sau khi đi 1 vòng qua vật cản sẽ được trả về giá trị như trước
float L_cicle1=86;
float L_Cicle2=118;
void MoppingMap(){      // Lau sàn theo quỹ đạo ziczac
  counter_1=0;
  counter_2=0;
  float velocity1=150;  //vận tốc bánh trái tỉ lệ vs quãng đường khi đi vòng
  float velocity2=110;  //vận tốc bánh phải tỉ lệ vs quãng đường khi đi vòng
  while(prev_counter<=setpoint_Distance_2){
    if(status==0){
      error1=setpoint_Distance_2-counter_1;
      int l=control_Motor_By_PID_up(error1);
      prev_counter=counter_1;
      lcd.setCursor(0,0);
      lcd.print(counter_1);
      lcd.setCursor(0,1);
      lcd.print(l);
      if(Measure_distance(TRIG_1,ECHO_1)<=20){
        status=1;   //chuyển sang trạng thái vòng qua vật cản
      }
    }
    else if(status==1){
      SpinLeft90();     // quay trái tìm đường đi
      Delay(200,stop);
      if(Measure_distance(TRIG_1,ECHO_1)>=100){
        status = 2;
      }
      else{
      while(counter_1<=L_cicle1&&counter_2<=L_Cicle2){
        motor_up(velocity1,motor1,motor2);
        motor_up(velocity2,motor3,motor4);
      }
      Delay(200,stop);
      SpinLeft90();
      counter_1=prev_counter-55;    //đặt lại counter để chuẩn bị về trạng thái đi thẳng
      status=1;    // chuyển về trạng thái đi thẳng
      }
  }
    else if(status==2){
      SpinRight180();     // quay trái tìm đường đi
      Delay(200,stop);
      if(Measure_distance(TRIG_1,ECHO_1)>=100){
        status = 3;
      }
      else{
      while(counter_1<=L_cicle1&&counter_2<=L_Cicle2){
        motor_up(velocity1,motor1,motor2);
        motor_up(velocity2,motor3,motor4);
      }
      Delay(200,stop);
      SpinRight90();
      counter_1=prev_counter-55;    //đặt lại counter để chuẩn bị về trạng thái đi thẳng
      status=0;    // chuyển về trạng thái đi thẳng
    }
  }
    else if(status==3){
      SpinRight90();
      Delay(200,stop);
      status=0;
    }
}

  Delay(100,stop);
    lcd.clear();
  counter_1=0;
  counter_2=0;

  while(counter_1<=setpoint_Distance_4){

    error1=setpoint_Distance_4-counter_1;
    int l=control_Motor_By_PID_TurnRight(error1);
    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);
  }

  Delay(100,stop);
    lcd.clear();
  counter_1=0;
  counter_2=0;

  while(counter_1<=setpoint_Distance_2){

    error1=setpoint_Distance_2-counter_1;
    int l=control_Motor_By_PID_up(error1);
    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);
  }

  Delay(100,stop);
    lcd.clear();
  counter_1=0;
  counter_2=0;

  while(counter_1<=setpoint_Distance_4){

    error1=setpoint_Distance_4-counter_1;
    int l=control_Motor_By_PID_TurnLeft(error1);
    lcd.setCursor(0,0);
    lcd.print(counter_1);
    lcd.setCursor(0,1);
    lcd.print(l);
  }

  Delay(100,stop);
    lcd.clear();
  counter_1=0;
  counter_2=0;

}

//                      Tránh vật cản đi theo quỹ đạo hình tròn
int length_1=100;
int length_2=100;

//          Module Bluetooth trong ứng dụng thực tế
void BlueTooth(){
  if(Serial.available()>0){
    int state= Serial.read();
    switch (state){
      case '1': 
        PaintingMap();
        Delay(100,stop);
        PaintingMap();
        Delay(10000,stop);
      break;
      case '2': 
        MoppingMap();
      break;
    }
  }
}
void loop(){
    PaintingMap();
    Delay(10000,stop);
}