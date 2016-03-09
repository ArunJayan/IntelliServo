//-------------------------------------------------------------
//--    IntelliServo
//--    I2C servo with temperature and current sensors
//--------------------------------------------------------------
//--    BQ
//--------------------------------------------------------------
//--    IntelliServo current sensing example by 
//--        Alvaro Ferran Cifuentes (alvaroferran)
//--------------------------------------------------------------
//--    Released on March 2016
//--    under the GPL v3
//--------------------------------------------------------------


#include "IntelliServo.h"
#include <Wire.h>

IntelliServo s1(0x01);

void setup(){
    Wire.begin();
    pinMode(13, OUTPUT);
    s1.writeAngle(90);
}

void loop(){

	if(s1.readCurrent()>15)
		digitalWrite(13,HIGH);
	else 
		digitalWrite(13,LOW);

}
 
