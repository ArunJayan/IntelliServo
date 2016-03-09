//-------------------------------------------------------------
//--    IntelliServo
//--    I2C servo with temperature and current sensors
//--------------------------------------------------------------
//--    BQ
//--------------------------------------------------------------
//--    IntelliServo sweep example by 
//--        Alvaro Ferran Cifuentes (alvaroferran)
//--------------------------------------------------------------
//--    Released on March 2016
//--    under the GPL v3
//--------------------------------------------------------------


#include "IntelliServo.h"
#include <Wire.h>

IntelliServo myServo(0x01);

void setup(){
    Wire.begin();
}


void loop(){

    myServo.writeAngle(20);
    delay(500);
   
    myServo.writeAngle(120);
    delay(500);
        

}
 
