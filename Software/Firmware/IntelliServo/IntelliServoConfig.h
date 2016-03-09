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

#ifdef FUTABA_S3003
    float maxAngle=200;
    float Kp=0.14, Kd=0, Ki= 0.0001;
#endif

#ifdef TURNIGY_1268HV  
    float maxAngle=200;
    float Kp=0.02, Kd=0, Ki= 0.0001;
#endif
