#include <SerialCommands.h>


SerialCommands sCmd; // Khai báo biến sử dụng thư viện Serial Command
 
const int LED_13 = 13;
 
const int ProductCode = 321; // mã sản phẩm của mình
 
void setup() {
  ​//Khởi tạo Serial ở baudrate 9600 (trùng với HOST
  ​Serial.begin(9600);
  
  ​//pinMode đèn LED 1
  ​pinMode(LED_13, OUTPUT);
  
  
  ​// Một số hàm trong thư viện Serial Comman
  
  ​sCmd.addCommand("PING",   ping);
  ​sCmd.addCommand("LED",   led);
  
  
}
 
void loop() {
  ​sCmd.readSerial();
  ​//Bạn không cần phải thêm bất kỳ dòng code nào trong hàm loop này c
}
 
// hàm LED_red sẽ được thực thi khi gửi hàm LED_RED
void led(){ 
  ​//Đoạn code này dùng để đọc TỪNG tham số. Các tham số mặc định có kiểu dữ liệu là "chuỗi
  ​char *arg;
  ​arg = sCmd.next();
  
  ​int value = atoi(arg); // Chuyển chuỗi thành s
  ​digitalWrite(LED_13, value);
  ​free(arg);
}
 
void ping() {
  ​char *arg;
  ​arg = sCmd.next();
  ​int code = atoi(arg);
  ​if (code == ProductCode) { // Yêu cầu đúng mã sản phẩ
    ​Serial.print(F("PING OK "));
    ​Serial.println(ProductCode);
  ​
  ​free(arg);
}