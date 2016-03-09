//-------------------------------------------------------------
//--    IntelliServo
//--    I2C servo with temperature and current sensors
//--------------------------------------------------------------
//--    BQ
//--------------------------------------------------------------
//--    Controlling an IntelliServo over serial
//--        Alvaro Ferran Cifuentes (alvaroferran)
//--------------------------------------------------------------
//--    Released on March 2016
//--    under the GPL v3
//--------------------------------------------------------------
//--    Usage:
//--    In serial monitor send
//--        'a'    : Set servo to manual, toggle angle reading
//--        'c'    : Toggle current consumption reading
//--        't'    : Print temperature reading
//--        (0-200): Move servo to specified angle  
//--------------------------------------------------------------

#include "IntelliServo.h"
#include <Wire.h>

IntelliServo myServo(0x01);

void setup(){
    Wire.begin();
    Serial.begin(115200);
    //myServo.changeAddress(0x2A); //Change servo's i2c address to 0x2A
}


void loop(){

    static int readAngle=-1;
    static int readCurrent=-1;

    if(Serial.available()){
        
        String incoming=readString();
        

        if (incoming.toInt()){
            myServo.writeAngle( incoming.toInt() );
            readAngle=-1;
        }

        if (incoming=="a"){
            myServo.disable();
            readAngle*=-1;
            readCurrent=-1;
        }

        if (incoming=="c"){
            readCurrent*=-1;
            readAngle=-1;
        }

        if (incoming=="t"){
            Serial.print("Temperature (ºC): ");
            Serial.println( myServo.readTemp() );
            readAngle=-1;
            readCurrent=-1;
        }


    }


    if(readAngle==1){
        Serial.print("Angle: ");
        Serial.println(myServo.readAngle());
    }

    if(readCurrent==1){
        Serial.print("Current: ");
        int current=myServo.readCurrent();
        Serial.print(current);
        if (current>20)
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
