// Motor
#define D0 8
#define D1 9
#define D2 7
#define D3 11
// CBSA trước
#define D4 2 
#define D5 3
//CBSA trái
#define D6 4
#define D7 5
//CBSA phải
#define D8 6
#define D9 10

void setup()
{
	pinMode(D0,OUTPUT);
	pinMode(D1,OUTPUT);
	pinMode(D2,OUTPUT);
	pinMode(D3,OUTPUT);
	pinMode(D4,INPUT);	//echo
	pinMode(D5,OUTPUT);
	pinMode(D6,INPUT);	//echo
	pinMode(D7,OUTPUT);
	pinMode(D8,INPUT);	//echo
	pinMode(D9,OUTPUT);
}


void move(int speed,int direction){
	if(direction==0){
		digitalWrite(D0,HIGH);
		analogWrite(D1,255-speed);
		digitalWrite(D2,HIGH);
		analogWrite(D3,255-speed);
	}
	else{
		digitalWrite(D0,LOW);
		analogWrite(D1,speed);
		digitalWrite(D2,LOW);
		analogWrite(D3,speed);
	}
}
void turn_Left(int speed,int direction){
	if(direction==0){
		digitalWrite(D0,HIGH);
		analogWrite(D1,0);
		digitalWrite(D2,HIGH);
		analogWrite(D3,255-speed);
	}
	else{
		digitalWrite(D0,HIGH);
		analogWrite(D1,0);
		digitalWrite(D2,LOW);
		analogWrite(D3,speed);
	}
}
void turn_Right(int speed,int direction){
		if(direction==0){
		digitalWrite(D0,HIGH);
		analogWrite(D1,255-speed);
		digitalWrite(D2,HIGH);
		analogWrite(D3,0);
	}
	else{
		digitalWrite(D0,LOW);
		analogWrite(D1,speed);
		digitalWrite(D2,LOW);
		analogWrite(D3,0);
	}
}

void spin_Left(int speed){

		digitalWrite(D0,LOW);
		analogWrite(D1,speed);
		digitalWrite(D2,HIGH);
		analogWrite(D3,255-speed);

}
void spin_Right(int speed){

		digitalWrite(D0,HIGH);
		analogWrite(D1,255-speed);
		digitalWrite(D2,LOW);
		analogWrite(D3,speed);
}


float get_distance1()		//cbsa phải
{
  long duration;
  float distanceCm;
  digitalWrite(D9, LOW);
  delayMicroseconds(2);
  digitalWrite(D9, HIGH);
  delayMicroseconds(10);
  digitalWrite(D9, LOW);
  
  duration = pulseIn(D8, HIGH, 5000);
 
  // convert to distance
  distanceCm = (float)duration / 29.1 / 2;
  return distanceCm;
}

float get_distance2()		//cbsa trước
{
  long duration;
  float distanceCm;
  digitalWrite(D5, LOW);
  delayMicroseconds(2);
  digitalWrite(D5, HIGH);
  delayMicroseconds(10);
  digitalWrite(D5, LOW);
  
  duration = pulseIn(D4, HIGH, 5000);
 
  // convert to distance
  distanceCm = (float)duration / 29.1 / 2;
  return distanceCm;
}
float get_distance3()		//cbsa trái
{
  long duration;
  float distanceCm;
  digitalWrite(D5, LOW);
  delayMicroseconds(2);
  digitalWrite(D5, HIGH);
  delayMicroseconds(10);
  digitalWrite(D5, LOW);
  
  duration = pulseIn(D4, HIGH, 5000);
 
  // convert to distance
  distanceCm = (float)duration / 29.1 / 2;
  return distanceCm;
}


int speed = 0;
int error=0
last_err = 0, setpoint = 20, u_t1 = 0;
float sample_time = 0.01;       // Khai báo thời gian lấy mẫu dt
float P = 0,I = 0,D = 0;

#define Kp  10.0
#define Ki  0.1
#define Kd  3.0      //Đặt các hệ số Kp, Ki, Kd

void calculate_PID1()
{

  P = Kp * error;                             //Tính P dựa trên Kp
  I += Ki * error * sample_time;              //Tính I dựa trên Ki
  D = Kd * ( last_err - error ) / sample_time;  //Tính D dựa trên Kd
  last_err = error;
  u_t1 = P + I + D;                            //Tính hàm truyền
}

void drive_motor1()
{
  // Giới hạn hàm truyền do tính toán u_t có thể vượt các giới hạn 255 và -255
  if( u_t > 255)
    u_t = 255;
  if(u_t < 0)
    u_t *= (255/20);  // Do khi vật cản < 20cm thì xe cần lùi nhanh hơn để tránh va chạm
  if( u_t < -255)
    u_t = -255;
    
	move(u_t,1);
}

int error1;
int error2;
void loop()
{
	error1=setpoint-get_distance1();
	calculate_PID1(error1);
	drive_motor(error2);
	if(get_distance2()==20){
		spin_Right();
		delay(1000);
	}
	error2=setpoint-get_distance3();
	calculate_PID2(error2);
	drive_motor(error2);
}
//em k đủ thời gian làm nốt ạ, bám tường trái tương tự bám tường phải

