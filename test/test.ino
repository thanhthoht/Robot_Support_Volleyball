#define S0 5 // khai báo các chân cảm biến màu sắc
#define S1 6
#define S2 7
#define S3 8
#define but_ 2
#define sensorOut 9
#define motorPin1 10
#define motorPin2 11
int step_number = 0;
#define trig 3
#define echo A1
Servo myServo;
int value_R = 0;
int value_B = 0;
int value_G = 0;
int colors[3][6];
int check1 = 0;
// lưu trữ các giá trị màu
int button(int pin)
{
 int buttonStatus = digitalRead(pin);
 return buttonStatus;
}
unsigned long timeR =0;
unsigned long timeG =0;
unsigned long timeB =0;
unsigned long timeM =0;
int readColor()
{

 digitalWrite(S2, LOW);
 digitalWrite(S3, LOW);
 // lọc màu đỏ trc khi đo
 if ((unsigned long)(millis() - timeR)>10)

 {
 value_R = pulseIn(sensorOut, LOW);
 timeR = millis();
 }
 digitalWrite(S2, HIGH);
 digitalWrite(S3, HIGH);
 // lọc màu xanh trc khi đo
 if ((unsigned long)(millis() - timeG)>10)
 {
 value_G = pulseIn(sensorOut, LOW);
 timeG = millis();
 }
 digitalWrite(S2, LOW);
 digitalWrite(S3, HIGH);

 if ((unsigned long)(millis() - timeB)>10)
 {
 value_B = pulseIn(sensorOut, LOW);
 timeB = millis();
 }

 for (int i=0; i<3; i++) {
 if (value_R>= colors[i][0] && value_R<= colors[i][1] &&
 value_G>= colors[i][2] && value_G <= colors[i][3] &&
 value_B>= colors[i][4] && value_B <= colors[i][5])
 {
 return i;
 }
 }
return -1;

}
int numR_value = 0;
int numG_value = 0;
int numB_value = 0;
int sum = 0;
int num_R = 0;
int num_G = 0;
int num_B = 0;
void calculator()
{
 for (int i=0 ; i<5; i++)
 {
 int c = readColor();
 if (-1 <c) {
 if (c == 0) numR_value = numR_value +1;
 if (c == 1) numG_value = numG_value +1;
 if (c == 2) numB_value = numB_value +1;
 delay(2);
 }
 }
 if (numR_value >2)
 {
 num_R = num_R+1;
 sum = sum+1;
 }
 if (numG_value >2)
{
 num_G = num_G+1;
 sum = sum +1;
 }
 if (numB_value >2)
 {
 num_B = num_B+1;
 sum = sum+1;
 }
 numR_value = 0;
 numG_value = 0;
 numB_value = 0;

// Serial.print((String)sum + " " +(String)num_R + " " (String)num_G + " " +(String)num_B);
}
void initColor()
{
 // xac' định màu đỏ
 colors[0][0] = 19; //Min R 14
 colors[0][1] =35; //Max R 18
 colors[0][2] = 57; //Min G 14
 colors[0][3] = 90; //Max G 18
 colors[0][4] = 57; //Min B 14
 colors[0][5] = 90; //Max B 18
 // xác định màu green
 colors[1][0] = 43; //Min R 14
 colors[1][1] = 70; //Max R 18
 colors[1][2] = 40; //Min G 14
 colors[1][3] = 75; //Max G 18
 colors[1][4] = 60; //Min B 14
 colors[1][5] = 95; //Max B 18
 // xác định màu blue
 colors[2][0] = 45; //Min R 14
 colors[2][1] = 75; //Max R 18
 colors[2][2] = 35; //Min G 14
 colors[2][3] =75; //Max G 18
 colors[2][4] = 23; //Min B 14
 colors[2][5] = 55; //Max B 18
}
int temp=0;
int sonar()
{

 for (int t=0; t<=2; t++)
 {

 unsigned long duration;
 int distance;
 digitalWrite(trig, 0);
 delayMicroseconds(2);
 digitalWrite(trig, 1);
 delayMicroseconds(5);
 digitalWrite(trig, 0);
 duration = pulseIn(echo, HIGH);
 distance = int(duration/2/29.412);
 if (distance< 4 ) {
 Serial.println("like");
 delay(100);
 return 1;
 }
// Serial.println("-"+(String)sonar()+"-");
 }
 return 0;

}
LiquidCrystal_I2C lcd(0x27, 16, 2); //
void printLCD(int sum, int num_R, int num_G, int num_B)
{
 lcd.clear();
 lcd.setCursor(1, 0);
 lcd.print("Tsp:");
 lcd.setCursor(5, 0);
 lcd.print(sum);
 lcd.setCursor(10, 0);
 lcd.print("R:");
 lcd.setCursor(12, 0);
 lcd.print(num_R);
 lcd.setCursor(3, 1);
 lcd.print("B:");
 lcd.setCursor(5, 1);
 lcd.print(num_B);
 lcd.setCursor(9, 1);
 lcd.print("G:");
 lcd.setCursor(11, 1);
 lcd.print(num_G);

}
void setup() {
 // put your setup code here, to run once:
 pinMode(S0, OUTPUT); // khai báo các chân cảm biến màu
 pinMode(S1, OUTPUT);
 pinMode(S2, OUTPUT);
 pinMode(S3, OUTPUT);
 pinMode(sensorOut, INPUT);
 digitalWrite(S0, HIGH);
 digitalWrite(S1, LOW);
 // 2 dòng trên để set cường độ đo 20 % năng lượng
pinMode(but,INPUT);
pinMode(but_, INPUT_PULLUP);
// đọc chân button
myServo.attach(4);
// khai báo chân cảm biến siêu âm
pinMode(trig, OUTPUT);
pinMode(echo, OUTPUT);
attachInterrupt(0, tat, CHANGE);
 Serial.begin(9600);

 lcd.init();
 lcd.backlight();
 initColor();

}
unsigned long time2=0;
void gat(byte state)
{
 switch (state)
 {
 case 1:
 for (int i=0; i<=180; i++)
 {
 time2 = millis();
 while ((unsigned long)(millis() - time2) <=5)
 {
 myServo.write(i);

 }
 }
 myServo.write(0);

 break;
 case 2:
 for (int i=180; i>=0; i--)
 {
 time2 = millis();
 while ((unsigned long)(millis() - time2) <=10)
 {
 myServo.write(i);

 }


 }
 myServo.write(0);
 break;
 }
}
void motor_Run()
{
 digitalWrite(motorPin1, HIGH);
 analogWrite(motorPin2, 50);
}
void motor_Stop()
{
 digitalWrite(motorPin1, LOW);
 analogWrite(motorPin2, 0);
}
int i=0;
void tat()
{
 i=0;
}
unsigned long timeLCDR = 0;
void loop()
{
 int color;
// motor(1);

 check1 = button(A0);
 if (check1 == HIGH)
 {
 i =1;
 check1 = LOW;
 while (i==1)
 {
 motor_Run();
 //Serial.println(i);
 color = readColor();

 if (color ==0 || color ==1 || color ==2)
 {
 switch(color)
 {
 case 0: // red

 calculator();
 printLCD(sum, num_R, num_G, num_B);
 delay(100);
 motor_Run();


 break;
 case 1: // Green

 calculator();
 printLCD(sum, num_R, num_G, num_B);
 delay(100);
 if (sonar() == 1)

 {
 delay(500);
 motor_Stop();

 gat(1);

 }
 Serial.println(sonar());
 //delay(100);
 motor_Run();
 break;
 case 2: // Blue
 calculator();
 printLCD(sum, num_R, num_G, num_B);
 delay(500);
 if (sonar() ==1) {
 delay(100);
 motor_Stop();
 //delay(1000);
 gat(2);
 }
 Serial.println(sonar());
 //delay(100);
 motor_Run();
 break;
 }
 }
 }
 motor_Stop();
 }

 }