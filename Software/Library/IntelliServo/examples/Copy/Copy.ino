//-------------------------------------------------------------
//--    IntelliServo
//--    I2C servo with temperature and current sensors
//--------------------------------------------------------------
//--    BQ
//--------------------------------------------------------------
//--    IntelliServo angle copying example by 
//--        Alvaro Ferran Cifuentes (alvaroferran)
//--------------------------------------------------------------
//--    Released on March 2016
//--    under the GPL v3
//--------------------------------------------------------------


#include "IntelliServo.h"
#include <Wire.h>

IntelliServo s1(0x01),s2(0x02);
IntelliServo servo[]={s1,s2};

void setup(){
    Wire.begin();
    delay(50);
}

void loop(){

	servo[0].writeAngle( servo[1].readAngle() );

	delay(50); 

}
 
