//--------------------------------------------------------------
//--	IntelliServo
//--	I2C servo with temperature and current sensors
//--------------------------------------------------------------
//--	BQ
//--------------------------------------------------------------
//--	Library created by 
//-- 		Alvaro Ferran Cifuentes (alvaroferran)
//--------------------------------------------------------------
//--	Released on March 2016
//--	under the GPL v3
//--------------------------------------------------------------

#ifndef IntelliServo_h
#define IntelliServo_h

#include "Arduino.h"
#include <Wire.h>  

#define INTELLISERVO_ADDRESS_I2C		0X0A         
#define INTELLISERVO_DISABLE			0x01                
#define INTELLISERVO_ANGLE_WRITE_LSB    0x02                  
#define INTELLISERVO_ANGLE_WRITE_MSB    0x03                 
#define INTELLISERVO_ANGLE_READ_LSB    	0x04                                                                          
#define INTELLISERVO_ANGLE_READ_MSB    	0x05
#define INTELLISERVO_CURRENT_READ_LSB	0x06
#define INTELLISERVO_CURRENT_READ_MSB   0x07
#define INTELLISERVO_TEMPERATURE_READ	0X08
// #define INTELLISERVO__LSB   0x0A


class IntelliServo{

	public:
		IntelliServo(uint8_t);
		void changeAddress(uint8_t);
		void disable(uint8_t db=1);
		void writeAngle(int);
		int16_t readAngle();
		int16_t readCurrent();
		int16_t readTemp();
		

	private:
		uint8_t INTELLISERVO_ADDRESS;
		void writeByte(uint8_t, uint8_t, uint8_t);
		uint8_t readByte(uint8_t, uint8_t);
		void readBytes(uint8_t, uint8_t, uint8_t, uint8_t *);

};
 
#endif
        
