//-------------------------------------------------------------
//--    IntelliServo
//--    I2C servo with temperature and current sensors
//--------------------------------------------------------------
//--    BQ
//--------------------------------------------------------------
//--    Library created by 
//--        Alvaro Ferran Cifuentes (alvaroferran)
//--------------------------------------------------------------
//--    Released on March 2016
//--    under the GPL v3
//--------------------------------------------------------------

#include "Arduino.h"
#include "IntelliServo.h"
#include <Wire.h>  

//PUBLIC

IntelliServo::IntelliServo(uint8_t address){
    INTELLISERVO_ADDRESS = address;
}


void IntelliServo::changeAddress(uint8_t address){
    writeByte(INTELLISERVO_ADDRESS, INTELLISERVO_ADDRESS_I2C, address);
}


void IntelliServo::disable(uint8_t db){
    writeByte(INTELLISERVO_ADDRESS, INTELLISERVO_DISABLE, db);
}


void IntelliServo::writeAngle(int angle){
    uint8_t myArray[2];
    myArray[0]=(angle >> 8) & 0xFF;
    myArray[1]=angle & 0xFF;    
    Wire.beginTransmission(INTELLISERVO_ADDRESS);  
    Wire.write(INTELLISERVO_ANGLE_WRITE_LSB);   
    Wire.write(myArray[0]);  
    Wire.write(myArray[1]);                
    Wire.endTransmission();
}


int16_t IntelliServo::readAngle(){
    uint8_t rawData[2];  
    readBytes(INTELLISERVO_ADDRESS, INTELLISERVO_ANGLE_READ_LSB, 2, &rawData[0]); 
    return (int16_t)  (rawData[0] << 8) | rawData[1];
}


int16_t IntelliServo::readCurrent(){
    uint8_t rawData[2];  
    readBytes(INTELLISERVO_ADDRESS, INTELLISERVO_CURRENT_READ_LSB, 2, &rawData[0]); 
    return (int16_t)  (rawData[0] << 8) | rawData[1];
}


int16_t IntelliServo::readTemp(){
    uint8_t rawData[2];  
    readBytes(INTELLISERVO_ADDRESS, INTELLISERVO_TEMPERATURE_READ, 2, &rawData[0]); 
    return (int16_t)  (rawData[0] << 8) | rawData[1];
}



//PRIVATE


void IntelliServo::writeByte(uint8_t address, uint8_t subAddress, uint8_t data){
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}



uint8_t IntelliServo::readByte(uint8_t address, uint8_t subAddress){
    uint8_t data; // `data` will store the register data   
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}


void IntelliServo::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest){  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); 
    }         
}
