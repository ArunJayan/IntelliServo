//-------------------------------------------------------------
//--    IntelliServo
//--    I2C servo with temperature and current sensors
//--------------------------------------------------------------
//--    BQ
//--------------------------------------------------------------
//--    Controlling two IntelliServos over serial
//--        Alvaro Ferran Cifuentes (alvaroferran)
//--------------------------------------------------------------
//--    Released on March 2016
//--    under the GPL v3
//--------------------------------------------------------------
//--    Usage:
//--    In serial monitor send
//--        'A/B'  : Select servo
//--        'a'    : Set servo to manual, toggle angle reading
//--        'c'    : Toggle current consumption reading
//--        't'    : Print temperature reading
//--        (0-200): Move servo to specified angle  
//--------------------------------------------------------------

#include "IntelliServo.h"
#include <Wire.h>

IntelliServo s1(0x01),s2(0x02);
IntelliServo servo[]={s1,s2};
int select=0;

void setup(){
    Wire.begin();
    Serial.begin(115200);
}


void loop(){

    static int readAngle=-1;
    static int readCurrent=-1;

    if(Serial.available()){
        
        String incoming=readString();
        
        if(incoming.substring(0, 1)=="A")
            select=0;
        else if(incoming.substring(0, 1)=="B") 
            select=1;


        if (incoming.substring(1).toInt()){
            servo[select].writeAngle( incoming.substring(1).toInt() );
            readAngle=-1;
        }

        if (incoming.substring(1)=="a"){
            servo[select].disable();
            readAngle*=-1;
            readCurrent=-1;
        }

        if (incoming.substring(1)=="c"){
            readCurrent*=-1;
            readAngle=-1;
        }

        if (incoming.substring(1)=="t"){
            Serial.print("Temperature (ºC): ");
            Serial.println( servo[select].readTemp() );
            readAngle=-1;
            readCurrent=-1;
        }


    }


    if(readAngle==1){
        Serial.print("Angle: ");
        Serial.println(servo[select].readAngle());
    }

    if(readCurrent==1){
        Serial.print("Current: ");
        int current=servo[select].readCurrent();
        Serial.print(current);
        if (current>10)
            Serial.print("      /_!_\\ WARNING /_!_\\");
        Serial.print("\n");
    }

}
 


String readString(){
    String inString ="";
    char inChar;
    while(Serial.available()>0){
        inChar =(char) Serial.read();
        inString+=inChar;
        delay(1);
    }
    return inString;
}
