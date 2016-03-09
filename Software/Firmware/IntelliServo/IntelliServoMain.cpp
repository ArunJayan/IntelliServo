//--------------------------------------------------------------
//--    IntelliServo
//--    I2C servo with temperature and current sensors
//--------------------------------------------------------------
//--    BQ
//--------------------------------------------------------------
//--    Firmware created by 
//--        Alvaro Ferran Cifuentes (alvaroferran)
//--------------------------------------------------------------
//--    Released on March 2016
//--    under the GPL v3
//--------------------------------------------------------------

//Select servo
#define FUTABA_S3003
//#define TURNIGY_1268HV



#include "mbed.h"
#include "USBSerial.h"
#include "IAP.h"
#include "IntelliServoConfig.h"
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//EEPROM values
#define     TARGET_ADDRESS  64 //First non-reserved address in EEPROM
#define     BYTE_SIZE       1

//USBSerial pc;
IAP     iap;

I2CSlave slave(P0_5, P0_4); //SDA, SCL

PwmOut mot1(P0_8);
PwmOut mot2(P0_9);
AnalogIn   pot(P0_11);
AnalogIn   current(P0_13);
AnalogIn   temperature(P0_14); 

void setAngle(int16_t);
int16_t getAngle();
int16_t getCurrent();
int16_t getTemp();
void changeAddress(uint8_t);
void setEepromByte(int, uint8_t);
uint8_t getEepromByte(int);
int eepromAddres=1; //After TARGET_ADDRESS



/********MAIN***************************************************************/

int main() {
    
    char inI2C[10];
    char outI2C[2];
    int8_t slaveRegister;
    int16_t angleIn, angleOut, currentOut, tempOut;
    bool angleSet=false;
    int8_t disable=1;
    
    mot2.period_us(33.33f);  // 30kHz 
    mot1.period_us(33.33f);  
    mot1.write(0.00f);
    mot2.write(0.00f);

    //Uncomment to reset default i2c address
    //setEepromByte(PLACE,(uint8_t) 0x01);

    uint8_t inValue, i2cAddress=0x02;
    inValue=getEepromByte(eepromAddres);
    if(inValue!= (uint8_t) 0 ) i2cAddress=inValue; 
    slave.address(i2cAddress<<1);    


    while(1){     
             
        int i = slave.receive();
        switch (i) {           
            
            case I2CSlave::WriteAddressed:      //Receive
                
                slave.read(inI2C, 10);
                slaveRegister=inI2C[0];

                switch(slaveRegister){
                    case 0x0A:  //Change address
                        changeAddress(inI2C[1]);
                    break;
                    
                    case 0x01:  //Disable motor
                        disable=inI2C[1];
                    break;
                    
                    case 0x02:  //Set angle
                        angleIn=(int16_t)  (inI2C[1] << 8) | inI2C[2];
                        disable=0; 
                        angleSet=true;
                    break;
                }
                
                
            break;
            
            
            case I2CSlave::ReadAddressed:       //Reply
                               
                switch(slaveRegister){
                    case 0x04:  //Get angle
                        outI2C[0]= (angleOut >> 8) & 0xFF;
                        outI2C[1]= angleOut & 0xFF;
                    break;
                    
                    case 0x06:  //Get current
                        outI2C[0]= (currentOut >> 8) & 0xFF;
                        outI2C[1]= currentOut & 0xFF;
                    break;
                    
                    case 0x08:  //Get temperature
                        outI2C[0]= (tempOut >> 8) & 0xFF;
                        outI2C[1]= tempOut & 0xFF;
                    break;
                }
                
                slave.write(outI2C, 2); 
                
            break;
            
        }
        for(int i = 0; i < 10; i++) inI2C[i] = 0;  
        
       
        
       //Keep setting the angle so the servo stays energized
       if(angleSet==true){ //Make sure servo only starts moving when a value has been sent
            if(disable==0)
                setAngle(angleIn);
            else {
                mot1.write(0.00f);
                mot2.write(0.00f);
            }
       }
        
       angleOut=getAngle();
       currentOut=getCurrent();
       tempOut=getTemp();
    
      
    }
    
}






/********SET ANGLE**********************************************************/
//PID setting based on Angel Espeso's example (http://roble.uno/control-pid-barra-y-bola-arduino/)

void setAngle(int16_t desiredAngle){

    desiredAngle=constrain(desiredAngle,-1,maxAngle);  //Limit legal angles
    
    int16_t currentAngle=0, lastAngle, error=0, lastError;
    int eCount=0;
    float I=0;
    while(1){
        
        wait_ms(1);
        
        //Read current angle 
        lastAngle=currentAngle;
        currentAngle=getAngle();
        
        //Error
        lastError=error;
        error=desiredAngle-currentAngle;
        
        //Calculate average speed
        float v[5];   //Speed vector
        for (int i=0; i<5; i++) // Move all speed values one space to the left to make space for the newest one
            v[i] =v [i+1];
        v[4] = (error-lastError); // Add last speed
        float vel=0;
        for (int i=0; i<5; i++)     // Average speed
            vel += v[i];
        vel /= 5;
        vel/=100;
                
        //I 
        if(abs(error)<10 && abs(error)>0.2)
            I+=error*Ki;
        else 
            I=0;
        
        //Voltage calculation
        float pwr=Kp*error+Kd*vel+I;
        
        //Motor control
        float speed=0;//0.06; 
        if(pwr>0){
            mot1.write(1-(abs(pwr)+speed)); 
            mot2.write(1); 
        }else {
            mot2.write(1-(abs(pwr)+speed)); 
            mot1.write(1);
        }
        
        //pc.printf("Pot Angle: %d   Pwr: %.2f  Current: %.2f\n" ,currentAngle ,abs(pwr)+speed ,current.read()*100/0.055);
    
        //Angle stable
        if(abs(currentAngle-lastAngle)<=1){     //Difference in angle instead of error to avoid blocking when angle is not recheable
            eCount++;
            if(eCount>5) 
                break;
        }
        
        
    }
}



/********GET ANGLE**********************************************************/

int16_t getAngle(){
    float aF=0;
    int16_t aI=0;
    for(int i=0; i<5; i++)
         aF+=(pot.read()*100-5)*2.439; 
    aI=(int16_t)constrain(aF/5,0,maxAngle);
    return aI;
}



/********GET CURRENT*********************************************************/

int16_t getCurrent(){
    float cF=0;
    int16_t cI=0;
    for(int i=0; i<5; i++)
         cF+=current.read()*100/0.055; 
    cI=(int16_t)cF/5;
    return cI;
}






/********GET TEMPERATURE****************************************************/

int16_t getTemp(){ 
    return (int16_t) (temperature.read()*3300-500)/10;
}




/********CHANGE ADDRESS*****************************************************/

void changeAddress(uint8_t address){
    setEepromByte(eepromAddres, address);
    slave.address(address<<1);  
}


/********EEPROM R&W*********************************************************/
//EEPROM acces code based on Dave Tech's example (https://developer.mbed.org/users/kstech/code/EepromTest/)

typedef union data {
    uint8_t b;
    char  s[1];
} myData; 

  
void setEepromByte(int place, uint8_t incomingByte){
    myData setByte;
    char someBytes[1];
    setByte.b = incomingByte;
    for(int i=0; i<sizeof(someBytes); i++)
        someBytes[i]=setByte.s[i];
    iap.write_eeprom( someBytes, (char*) (TARGET_ADDRESS+place), BYTE_SIZE );
} 
 
uint8_t getEepromByte(int place){
    myData getByte;
    char someBytes[1];
    iap.read_eeprom( (char*)(TARGET_ADDRESS+place), someBytes, BYTE_SIZE );
    for(int i=0; i<sizeof(someBytes); i++)
        getByte.s[i]=someBytes[i];
    return getByte.b;
}
