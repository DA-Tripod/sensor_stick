#include <DebugUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
#include <Wire.h>

FreeSixIMU imu = FreeSixIMU();

//ADXL  
#define ADXL (0x53)    //ADXL345 device address 
#define TO_READ_ACCEL (6)        //num of bytes we are going to read each time (two bytes for each axis) 
#define TO_READ_OFFSET (3)
#define POWER_CTL_ADXL (0x2D)   //Power Control Register
#define DATA_FORMAT_ADXL (0x31)
#define ACCELADDRESS (0x32)    //first axis-acceleration-data register on the ADXL345
#define OFFSETX (0x1E)  //offset-registers
#define OFFSETY (0x1F)
#define OFFSETZ (0x20)

float gs[6];
float ypr[3];

//ADXL
byte accelValues[TO_READ_ACCEL] ;    //6 bytes buffer for saving data read from the device
byte offsetValues[TO_READ_OFFSET];
double offX, offY, offZ;
float vx=0.0, vy=0.0, vz=0.0, sx=0.0, sy=0.0, sz=0.0, vxOld, axOld;
float ax=0.0, ay=0.0, az=0.0, axtp=0.0;
int counterVx=0, counterAx;

float time = 0.062;

float ta=0, tn;

void setup() {
  Serial.begin(9600);
  delay(10);
  Wire.begin();
  delay(10);
  
  imu.init();
  delay(1000);
  
  /*writeTo(ADXL, OFFSETX, 0);
  writeTo(ADXL, OFFSETY, 0);
  writeTo(ADXL, OFFSETZ, 0);*/
  
  delay(2000);
  
   for(int i=0;i<100;i++){
    
    imu.getValues(gs);
   
    ax=ax+gs[0];
    ay=ay+gs[1];
    az=az+gs[2];
  } 
  
  offX = ax/100.0;
  offY = ay/100.0;
  offZ = az/100.0;
  
  /*
  //Offset
  writeTo(ADXL, OFFSETX, offX);
  writeTo(ADXL, OFFSETY, offY);
  writeTo(ADXL, OFFSETZ, offZ);
  */
    
  delay(50);
}

void loop() {
  time=(millis()-ta)/1000.0;
  // Serial.println(time);
  ta=millis();  
  
  if(gs[3] > 15 || gs[3] < (-15) || gs[4] > 15 || gs[4] < (-15) || gs[5] > 15 || gs[5] < (-15)){    // falls Drehung
    // Code für Schrittmotoransteuerung der Roboterhand
    
    for(int i=0;i<100;i++){    // neuen Offset bestimmen
      imu.getValues(gs);
     
      ax=ax+gs[0];
      ay=ay+gs[1];
      az=az+gs[2];
    } 
  
    offX = ax/100.0;
    offY = ay/100.0;
    offZ = az/100.0;
    delay(50);
  }
  else{      // falls keine Drehung
    imu.getValues(gs);
    imu.getYawPitchRoll(ypr);
    
    
    axOld = ax;
    vxOld = vx;
 
    ax = ((4.0*(gs[0]-offX)/1024.0)*9.81)*1000.0;
    ay = (4.0*gs[1]-offY/1024.0)*9.81;
    az = (4.0*gs[2]-offZ/1024.0)*9.81;
    
    if((axOld - ax)>(-75) && (axOld - ax) < 75){    // Stillstand?
      counterAx++;
      
      if(counterAx == 5){    // neuen Offset berechnen
        counterAx = 0;
        
        for(int i=0;i<100;i++){
          imu.getValues(gs);
         
          ax=ax+gs[0];
          ay=ay+gs[1];
          az=az+gs[2];
        } 
    
        offX = ax/100.0;
        offY = ay/100.0;
        offZ = az/100.0;
        delay(50);
      }
    }else{    // wenn kein Stillstand -> counterAx auf 0 setzen
      counterAx = 0;
    }
    
    //axtp=axtp*0.98+ax*0.02;
    
    //ax -= sin(ypr[1])*9810;  
  
    if(ax>100||ax<(-100)){    // Beschleunigungswerte außerhalb des Sensor-Schwingwerte-Bereichs?
      vx += ax*time;
    }
  
    if((vxOld - vx)<3 && (vxOld - vx) > (-3)){    // alte Geschwindigkeit und neue Geschwindigkeit annähernd die selbe?
      counterVx++;
      
      if(counterVx == 4){    // wenn 4-mal hintereinander beinahe die selbe Geschwindigkeit
        vx = 0;
        counterVx = 0;
      }
    }else{    // wenn nicht 4-mal hintereinander beinahe die selbe Geschwindigkeit -> counterVx auf 0 setzen
      counterVx = 0;
    }
      
    if(vx>150||vx<-150){    // Geschwindigkeit außerhalb des Sensor-Schwingwerte-Bereichs und der Aufintegrationsfehler?
      sx += vx*time;
    }    
  }
 
  
  //--------------------------------------
  /*
  if((ay>0.1)||(ay<(-0.1))){
    vy += ay*time;
  }
  if(vy>0.1||vy<-0.1){
    sy +=vy*time;
  }
  
  if((az>0.1)||(az<(-0.1))){
    vz += az*time;
  }
  if(vz>0.1||vz<-0.1){
    sz +=vz*time;
  }
  */
  
  Serial.print("offX:");
  Serial.print(offX);
  //Serial.print("    aRaw:");
  //Serial.print(gs[0]);
  Serial.print("    aX:");
  Serial.print(ax);
  /*Serial.print("Y:");
  Serial.println(ay);
  Serial.print("Z:");
  Serial.println(az);*/
  
  Serial.print("    vX:");
  Serial.print(vx);
  //Serial.print("    vxOld:");
  //Serial.print(vxOld);
  Serial.print("    sX:");
  Serial.println(sx);
  /*Serial.print("OrtY:");
  Serial.println(sy);
  Serial.print("OrtZ:");
  Serial.println(sz);
  
  //Serial.print("Yaw:");
  //Serial.println(ypr[0]);*/
  /*Serial.print("   Pitch:");
  Serial.print(ypr[1]);
  Serial.print("   Roll:");
  Serial.println(ypr[2]); */
  
  delay(50);
}

//---------------- Functions 
//Writes val to address register on device 
void writeTo(int device, byte address, byte val) { 
  Wire.beginTransmission(device); //start transmission to device  
  Wire.write(address);        // send register address 
  Wire.write(val);        // send value to write 
  Wire.endTransmission(); //end transmission 
} 
  
//reads num bytes starting from address register on device in to buff array 
void readFrom(int device, byte address, int num, byte buff[]) { 
  Wire.beginTransmission(device); //start transmission to device  
  Wire.write(address);        //sends address to read from 
  Wire.endTransmission(); //end transmission 
  
  Wire.beginTransmission(device); //start transmission to device 
  Wire.requestFrom(device, num);    // request 6 bytes from device 
  
  int i = 0; 
  while(Wire.available())    //device may send less than requested (abnormal) 
  {  
    buff[i] = Wire.read(); // receive a byte 
    i++; 
  } 
  Wire.endTransmission(); //end transmission 
}
