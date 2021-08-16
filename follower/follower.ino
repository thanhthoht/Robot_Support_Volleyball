#define TRIG_PIN 2
#define ECHO_PIN 12
#define TIME_OUT 5000
#define SETPOINT 20
#define OFFSET 8
void setup() {
  pinMode(8,OUTPUT);//Direction w1
  pinMode(7,OUTPUT);//Direction w2
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
}
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

void control_bot(int v_wheel)
{
  if (v_wheel > 0 )
  {
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    analogWrite(3,v_wheel);
    analogWrite(6,v_wheel);
  }else
  {
    digitalWrite(8, HIGH);
    digitalWrite(7, HIGH);
    analogWrite(3,-v_wheel);
    analogWrite(6,-v_wheel);
  }
}

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
  control_bot(v_wheel);
  
  prev_et = et;
  delay(10);
  }

void loop() {
  float error = SETPOINT - get_distance();
  Serial.println(get_distance());
  PID(error);
}
