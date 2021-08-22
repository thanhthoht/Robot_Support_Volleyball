

#define in1 12
#define in2 13
#define enA 8
#define in3 14
#define in4 15
#define enB 7
int encoder_L=5;
int encoder_R=3;

volatile unsigned int counter_1;
int rpm_1;
volatile unsigned int counter_2;
int rpm_2;

void setup()
{
	pinMode(in1,OUTPUT);
	pinMode(in2,OUTPUT);
	pinMode(enA,OUTPUT);
	pinMode(in3,OUTPUT);
	pinMode(in4,OUTPUT);
	pinMode(enB,OUTPUT);

	//encoder trái
	pinMode(encoder_L,INPUT);
		digitalWrite(encoder_L, HIGH);
  		attachInterrupt(0, countpulse_1, RISING);
  	//encoder phải
	pinMode(encoder_R,INPUT);
		digitalWrite(encoder_R, HIGH);
 		attachInterrupt(1, countpulse_2, RISING);

}

void countpulse_1(){
        counter_1++;
}
void countpulse_2(){
        counter_2++;
}
double encoder_1(){
  static uint32_t previousMillis;
  if (millis() - previousMillis >= 1000) {
            rpm_1 = (counter_1/20)*60;          
            counter_1 = 0;
            previousMillis += 1000;
  }
  return rpm_1;
}

double encoder_2(){
  static uint32_t previousMillis;
  if (millis() - previousMillis >= 1000) {
            rpm_2 = (counter_2/20)*60;          
            counter_2 = 0;
            previousMillis += 1000;
  }
  return rpm_2;
}

void Motor_1(int v){
	digitalWrite(in1,HIGH);
	digitalWrite(in2,LOW);
	analogWrite(enA,v);
}
void Motor_2(int v){
	digitalWrite(in3,HIGH);
	digitalWrite(in4,LOW);
	analogWrite(enB,v);
}

//												Tính RPM (số vòng quay)

//												Thiết lập bộ điều khiển PID
double Kp = 100;
double Ki = 2;
double Kd = 3;
double dt = 0.01;
double sum_err=0;
double prev_et=0;
double PID_Motor(double error)
{
  sum_err += error * dt;
  double P = Kp * error;
  double I = Ki * sum_err;
  double D = Kd * (error - prev_et)/dt;

  double ut = P + I + D;
  return ut;
}
//
//
//


void loop()
{
	 Motor_1(170);
	 Motor_2(170);
}

