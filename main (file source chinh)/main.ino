#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define TRIG_PIN 7
#define ECHO_PIN 8
#define TIME_OUT 5000
#define SETPOINT 20
#define OFFSET 8
#define in1 12
#define in2 13
#define in3 14
#define in4 15
#define en1 18
#define en2 19
#define en3 16
#define en4 17

int valor;
long iii=0;
int SA=5;
int speed;
LiquidCrystal_I2C lcd(0x3F,44,2);

void setup()
{
  speed=150;
	pinMode(in1,OUTPUT);  
	pinMode(in2,OUTPUT);  
	pinMode(in3,OUTPUT);
	pinMode(in4,OUTPUT);
	pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
	pinMode(en3,INPUT);
  pinMode(en4,INPUT);
}



// Hàm hiện LCD
void HienLcd(int speed){
	 lcd.begin();
	 lcd.backlight();
  	for(int i=0;i<10;i++){
  		lcd.print(speed);
  		lcd.print("m/s");
      lcd.setCursor(0,1);
      lcd.print(analogRead(en3));
      lcd.setCursor(0,2);
      lcd.print(analogRead(en4));
      delay(1000);
  		lcd.clear();
  		delay(10);
  	}
}

//cảm biến siêu âm đo khoảng cách
float prev_distanceCm = 0;
float get_distance()
{
  long duration;
  float distanceCm;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH, TIME_OUT);
 
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

//hàm điều khiển motor tiến
void motortien(int a1,int a2,int a3,int a4,int speed){
	digitalWrite(a1, HIGH);
	analogWrite(a2, 255-speed);
	digitalWrite(a3,HIGH);
	analogWrite(a4, 255-speed);
}

//Hàm PID
float sum_err = 0;
float prev_et = SETPOINT - get_distance();

void PID(float et)
{
  int Kp = 100;
  int Ki = 2;
  int Kd = 3;
  float dt = 0.01;
  
  sum_err += et * dt;
  float P = Kp * et;
  float I = Ki * sum_err;
  float D = Kd * (et - prev_et)/dt;

  float ut = P + I + D;
  int v_wheel = 70 + (int)ut;

  if (v_wheel > 255) v_wheel = 255;
  if (v_wheel < -255) v_wheel = -255;

  prev_et = et;
  delay(10);
}


void loop()
{                 
  	motortien(in1,in2,in3,in4,speed);
  	HienLcd(speed);

}