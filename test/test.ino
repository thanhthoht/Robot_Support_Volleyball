#include <LiquidCrystal_I2C.h>
#include <Wire.h>
LiquidCrystal_I2C lcd(0x3F,44,2);

#define motor3 7
#define motor4 8
#define motor1 13
#define motor2 12
#define en1 11
#define en2 9
#define TRIG 20
#define ECHO 21

int encoder_L=5;
int encoder_R=3;

volatile unsigned int counter_1;
int rpm_1;
volatile unsigned int counter_2;
int rpm_2;

void setup()
{
	Serial.begin(9600);
	pinMode(motor1,OUTPUT);
	pinMode(motor2,OUTPUT);
	pinMode(motor3,OUTPUT);
	pinMode(motor4,OUTPUT);
	pinMode(en1,OUTPUT);
	pinMode(en2,OUTPUT);

	//encoder trái
	pinMode(encoder_L,INPUT);
		digitalWrite(encoder_L, HIGH);
  		attachInterrupt(0, countpulse_1, RISING);
  	//encoder phải
	pinMode(encoder_R,INPUT);
		digitalWrite(encoder_R, HIGH);
 		attachInterrupt(1, countpulse_2, RISING);

 	lcd.begin();
}

void countpulse_1(){
        counter_1++;
}
void countpulse_2(){
        counter_2++;
}

void motor_up(int in1,int in2,int en,int speed){
	digitalWrite(in1,HIGH);
	digitalWrite(in2,LOW);
	analogWrite(en,speed);
}
void motor_down(int in1,int in2,int en,int speed){
	digitalWrite(in1,LOW);
	digitalWrite(in2,HIGH);
	analogWrite(en,speed);
}

//												Tính RPM (số vòng quay)

//												Thiết lập bộ điều khiển PID
float rKp = 1;
float rKi = 0.5;
float rKd = 0.05;
float rdt = 10;
float rsum_err=0;
float rprev_et=0;
float Right_PID_Motor(float rerror)
{
  rsum_err += rerror * rdt;
  float P = rKp * rerror;
  float I = rKi * rsum_err;
  float D = rKd * (rerror - rprev_et)/rdt;
  float ut = P + I + D;
  return ut;
}

float lKp = 1;
float lKi = 0.5;
float lKd = 0.05;
float ldt = 10;
float lsum_err=0;
float lprev_et=0;
float Left_PID_Motor(float lerror)
{
  lsum_err += lerror * ldt;
  float P = lKp * lerror;
  float I = lKi * lsum_err;
  float D = lKd * (lerror - lprev_et)/ldt;
  float ut = P + I + D;
  return ut;
}

int control_Motor_By_PID_up(float rerror,float lerror){
  float velocity1=104+Right_PID_Motor(rerror);  
  float velocity2=104+Left_PID_Motor(lerror);  
  // vận tốc gốc để Motor encoder quay trong proteus là 104
  motor_up(motor1,motor2,en1,200);                                  
  motor_up(motor3,motor4,en2,velocity1);

}

float setpoint_Distance_1=100;
float rerror,lerror;
void loop()
{
	while(counter_1<=100&&counter_2<=100){
		lcd.print(counter_1);
		lcd.setCursor(0,1);
		lcd.print(counter_2);
		rerror=setpoint_Distance_1-counter_1;
		lerror=setpoint_Distance_1-counter_2;
		control_Motor_By_PID_up(rerror,lerror);
	}
}

