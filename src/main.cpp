#include <Arduino.h>
#include<Wire.h>
#include<vector>
//---I2C--//
#define ADDR 0x68
#define SDA 8
#define SCL 9
//---REGISTROS---//
#define PWM_MGMT_1    0x6B  //Reg 107:Power Management 1, pag 40
#define ACCEL_CONFIG  0x1C  //Reg 28: Configuración del acelerometro, pag 15 
#define ACCEL_XOUT_H  0x3B  //Reg 59: Salida HIGH X  
#define ACCEL_XOUT_L  0x3C  //Reg 60: Salida LOW X
#define ACCEL_YOUT_H  0x3D  //Reg 61: Salida HIGH Y
#define ACCEL_YOUT_L  0x3E  //Reg 62: Salida LOW Y
#define ACCEL_ZOUT_H  0x3F  //Reg 63: Salida HIGH Z
#define ACCEL_ZOUT_L  0x40  //Reg 64: Salida LOW Z
#define GYRO_CONFIG   0X1A  //Reg 27: Configuración del giroscopio, pag 14
#define GYRO_XOUT_H   0X43  //Reg 67: Salida HIGH X
#define GYRO_XOUT_L   0X44  //Reg 68: Salida LOW X
#define GYRO_YOUT_H   0X45  //Reg 69: Salida HIGH Y 
#define GYRO_YOUT_L   0X46  //Reg 70: Salida LOW Y 
#define GYRO_ZOUT_H   0X47  //Reg 71: Salida HIGH Z
#define GYRO_ZOUT_L   0X48  //Reg 72: Salida LOW Z
//---WAKE-UP---//
void wakeUp(){
  Wire.beginTransmission(PWM_MGMT_1);
  Wire.write(0);
  if (Wire.endTransmission(true)!=0){
    Serial.println("Wake-up failed");
  }else{
    Serial.println("Awake");
  }
}
//---ACCEL CONFIG---//
/*
0 = +/- 2g
1 = +/- 4g
2 = +/- 8g
3 = +/- 16g 
*/ 
void configAccel(uint8_t reg,uint8_t value){
  if((reg!=ACCEL_CONFIG)&&(value!=0x00||0x01||0x02||0x03)){    
    Serial.print("Such configuration can not be made");       
  }else{                                                       
    Wire.beginTransmission(ADDR);  
    Wire.write(reg);                            
    Wire.write(value); 
  }
  if (Wire.endTransmission(true)!=0){
    Serial.println("Config failled");
  } 
  }
//---GYRO CONFIG---//
/*
0 = +/- 250°/s
1 = +/- 500°/s
2 = +/- 1000°/s
3 = +/- 2000°/s
*/
void configGyro(uint8_t reg, uint8_t value){
  if((reg!=GYRO_CONFIG)&&(value!=0x00||0x01||0x02||0x03)){    
    Serial.print("Such configuration can not be made");
  }else{
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(value);
  }
  if (Wire.endTransmission(true)!=0){
    Serial.println("Config Failled");
  }
} 
//---INICIALIZACION---//
void init(){
  configAccel(ACCEL_CONFIG,0X00);
  delay(500);
  configGyro(GYRO_CONFIG, 0X00);
  delay(500);
  Serial.println("Sensor configurated");
}
//---LECTURA---//
int16_t Read(uint8_t msb_reg, uint8_t lsb_reg){
  Wire.beginTransmission(ADDR);
  Wire.write(msb_reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR,2,true);
  while (Wire.available()<2){

  }
  int16_t MSB = Wire.read();
  int16_t LSB = Wire.read();
  return(MSB<<8)|LSB;
}
//---LECTURA Y CONVERSION ACELEROMETRO---//
void ACCEL(){
  float Ax = Read(ACCEL_XOUT_H,ACCEL_XOUT_L)/16384.0;
  float Ay = Read(ACCEL_YOUT_H,ACCEL_YOUT_L)/16384.0;
  float Az = Read(ACCEL_ZOUT_H,ACCEL_ZOUT_L)/16384.0;
  Serial.print("X: "); Serial.print(Ax); Serial.print(" g");
  Serial.print(" | Y: "); Serial.print(Ay); Serial.print(" g");
  Serial.print(" | Z: "); Serial.print(Az); Serial.println(" g");
}
//---LECTURA Y CONVERSION GIROSCOPIO---//
void GYRO(){
  float Gx = Read(GYRO_XOUT_H,GYRO_XOUT_L)/131.0;
  float Gy = Read(GYRO_YOUT_H,GYRO_YOUT_L)/131.0;
  float Gz = Read(GYRO_ZOUT_H,GYRO_ZOUT_L)/131.0;
  Serial.print("X: "); Serial.print(Gx); Serial.print("°/s");
  Serial.print(" | Y: ");Serial.print(Gy); Serial.print("°/s");
  Serial.print(" | Z: ");Serial.print(Gz); Serial.println("°/s");
  
}
void setup() {
  Serial.begin(15200);  
  Wire.begin(SDA, SCL);
  delay(100);
  wakeUp();
  delay(1000);
  init();
  delay(100);
}
void loop() {
  ACCEL();
  GYRO();

}


