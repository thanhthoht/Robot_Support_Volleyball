#include <LiquidCrystal_I2C.h>
#define motor3 7
#define motor4 8
#define motor1 13
#define motor2 12
#define TRIG 20
#define ECHO 21
#define TIME_OUT 5000
int SA=5;
int SB=11;
long counter_1=0;
int M1=HIGH,M2=LOW;


LiquidCrystal_I2C lcd(0x3F,44,2);

void setup()
{
	attachInterrupt(0, giro, RISING);
	pinMode(SA,INPUT);
	attachInterrupt(1,readEncoder_L,RISING);
	pinMode(motor1,OUTPUT);
	pinMode(motor2,OUTPUT);
	pinMode(motor3,OUTPUT);
	pinMode(motor4,OUTPUT);
	pinMode(TRIG,OUTPUT);
	pinMode(ECHO,INPUT);
	lcd.begin();
}

void giro(){
	M1=!M1;
	M2=!M2;
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

// Điều khiển motor
void motor_up(int in1,int in2,int speed){
	digitalWrite(in1,HIGH);
	analogWrite(in2,255-speed);
}
void motor_back(int in1,int in2,int speed){
	digitalWrite(in1,LOW);
	analogWrite(in2,speed);
}
void stop(){
	digitalWrite(motor1,LOW);
	digitalWrite(motor2,LOW);
	digitalWrite(motor3,LOW);
	digitalWrite(motor4,LOW);
}
void turn_Left(int speed){				// Motor 1-2 lùi, Motor 3-4 tiến
	motor_back(motor1,motor2,speed);
	motor_up(motor3,motor4,speed);
}
void turn_Right(int speed){				// Motor 1-2 tiến, Motor 3-4 lùi
	motor_back(motor3,motor4,speed);
	motor_up(motor1,motor2,speed);
}

//																Thiết lập bộ điều khiển PID cho Motor
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
	float velocity=150+PID_Motor(error);	// vận tốc gốc để Motor encoder trong proteus là 127
							// tức là lên 128 motor sẽ quay được.
	motor_up(motor1,motor2,velocity);
	motor_up(motor3,motor4,velocity);
	return velocity;
}




//																			Thiết lập bộ điều khiển PID cho góc quay
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
//												Bộ PID dừng trước vật cản 20 cm

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
float Kp_d = 1;
float Ki_d = 0.5;
float Kd_d = 0.5;
float dt_d = 0.01;
float sum_err_d=0;
float prev_et_d=0;
float PID_Duration(float error)
{
  sum_err_d += error * dt_d;
  float P = Kp_d * error;
  float I = Ki_d * sum_err_d;
  float D = Kd_d * (error - prev_et_d)/dt_d;
  float ut = P + I + D;
  prev_et_d=error;
  return ut;
}


void printRPM(){
	lcd.setCursor(0,0);
	lcd.print("c");
	lcd.setCursor(0,1);
	lcd.print(counter_1);
}
int setpoint_Distance_1=100;
int setpoint_Angle_1=90;
int setpoint_Distance_2=60;
int setpoint_Stop_Barrier=20;
float error1;		// Quãng đường
float error2;		// Góc quay
float error3;		// Dừng trước vật cản
void loop()
{
	error1=setpoint_Distance_1-counter_1;
	error2=setpoint_Angle_1;
		motor_back(motor1,motor2,200);
		motor_back(motor3,motor4,200);	
	while(counter_1<=100){
		error1=setpoint_Distance_1-counter_1;
		int l=control_Motor_By_PID_up(error1);
		lcd.setCursor(0,1);
		lcd.print(counter_1);
		lcd.setCursor(0,2);
		lcd.print(l);
		if(counter_1==100){
			break;
		}
	}

	// else{
	// 		if(error2<=90){
	// 			motor_back(motor1,motor2,200);
	// 			motor_back(motor3,motor4,200);
	// 		}
	// }

}


