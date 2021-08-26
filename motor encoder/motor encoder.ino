
#include <LiquidCrystal_I2C.h>

#include <cppQueue.h>

LiquidCrystal_I2C lcd(0x3F,44,2);

cppQueue q(sizeof(int),5,FIFO);

void setup()
{
	lcd.begin();
	lcd.backlight();
}

void loop()
{
	int out;
	for(int i=0;i<100;i++){
		lcd.print(i);
		delay(200);
	}
}