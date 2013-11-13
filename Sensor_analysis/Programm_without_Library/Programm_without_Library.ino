//Quelle: http://www.dfrobot.com/wiki/index.php/Triple_Axis_Accelerometer_Breakout_-_ADXL345_(SKU:SEN0032) !!!
//QUelle: https://www.sparkfun.com/tutorials/240 !!!!

#include <Wire.h> 
#include <math.h>
 
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

//ITG
#define SMPLRT_DIV (0x15)
#define DLPF_FS (0x16)
#define INT_CFG (0x17)
#define PWR_MGM (0x3E)
#define ITG (0x68)
#define TO_READ_GYRO (6)

//ADXL
byte accelValues[TO_READ_ACCEL] ;    //6 bytes buffer for saving data read from the device
byte offsetValues[TO_READ_OFFSET];
double offX, offY, offZ;
double vx=0, vy=0, vz=0, sx=0, sy=0, sz=0;

//ITG
byte gyroValues[TO_READ_GYRO];
double gyroX, gyroY, gyroZ;

double angleX, angleY, angleZ;
const double time = 0.07;

int ta, tn;

void setup() 
{ 
  double arrayX[4], arrayY[4], arrayZ[4], ax=0, ay=0, az=0;
  
  Wire.begin();        // join i2c bus (address optional for master) 
  Serial.begin(9600);  // start serial for output 
  
  //ADXL
  writeTo(ADXL, DATA_FORMAT_ADXL, 0);  // +/- 2g range
  writeTo(ADXL, POWER_CTL_ADXL, 8);    // measurement mode
  
  writeTo(ADXL, OFFSETX, 0);
  writeTo(ADXL, OFFSETY, 0);
  writeTo(ADXL, OFFSETZ, 0);
  /*
  //Turning on the ADXL345 
  writeTo(ADXL, 0x2D, 0);       
  writeTo(ADXL, 0x2D, 16); 
  writeTo(ADXL, 0x2D, 8); 
  */
  
  delay(2000);
  
   for(int i=0;i<100;i++){
    readFrom(ADXL, ACCELADDRESS, TO_READ_ACCEL, accelValues); //read the acceleration data from the ADXL345
    arrayX[i] = (((int)accelValues[1]) << 8) | accelValues[0];    
    arrayY[i] = (((int)accelValues[3])<< 8) | accelValues[2]; 
    arrayZ[i] = (((int)accelValues[5]) << 8) | accelValues[4];
    delay(20);
   
    ax=ax+arrayX[i];
    ay=ay+arrayY[i];
    az=az+arrayZ[i];
  } 
  
  offX = -(ax/400);
  offY = -(ay/400);
  offZ = -((az-256)/400);
  
  //Offset
  writeTo(ADXL, OFFSETX, offX);
  writeTo(ADXL, OFFSETY, offY);
  writeTo(ADXL, OFFSETZ, offZ);
 
  //ITG
  writeTo(ITG, PWR_MGM, 0x80);    // reset
  writeTo(ITG, SMPLRT_DIV, 0x00);    
  writeTo(ITG, DLPF_FS, 0x1C);    //std: 18
  writeTo(ITG, INT_CFG, 0x00);    // no interrupts
  writeTo(ITG, PWR_MGM, 0x00);    // normal operation
 
  delay(50);
} 
  
void loop() 
{   
  ta=millis();
  //ADXL
  double ax, ay, az, axg, ayg, azg;
  int arrayX[4], arrayY[4], arrayZ[4];
  double angleXg, angleYg, angleZg, angleXaccel, angleYaccel, angleZaccel;
 
  //ITG
  int gyroAddress = 0x1D;
  double gX, gY, gZ;
   
  //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!! 
  //thus we are converting both bytes in to one int 
  
  /* 
  //ADXL
  for(int i=0; i<4; i++){
    readFrom(ADXL, ACCELADDRESS, TO_READ_ACCEL, accelValues); //read the acceleration data from the ADXL345
    arrayX[i] = (((int)accelValues[1]) << 8) | accelValues[0];    
    arrayY[i] = (((int)accelValues[3])<< 8) | accelValues[2]; 
    arrayZ[i] = (((int)accelValues[5]) << 8) | accelValues[4];
    delay(12);
  }
  
  sortArray(arrayX, 4);
  sortArray(arrayY, 4);
  sortArray(arrayZ, 4);
   
  axg = (arrayX[1]+arrayX[2])/2;
  ayg = (arrayY[1]+arrayY[2])/2;
  azg = (arrayZ[1]+arrayZ[2])/2;
  */
  
  readFrom(ADXL, ACCELADDRESS, TO_READ_ACCEL, accelValues); //read the acceleration data from the ADXL345
  delay(15);
  axg = (((int)accelValues[1]) << 8) | accelValues[0];    
  ayg = (((int)accelValues[3])<< 8) | accelValues[2]; 
  azg = (((int)accelValues[5]) << 8) | accelValues[4];
  
  ax = (4.0*axg/1024.0)*9.81;
  ay = (4.0*ayg/1024.0)*9.81;
  az = (4.0*azg/1024.0)*9.81;
  
  if((ax>0.2)||(ax<(-0.2))){
    vx = vx + ax*time;
    sx = sx + vx*time;
  }
  if((ay>0.2)||(ay<(-0.2))){
    vy = vy + ay*time;
    sy = sy + vy*time;
  }
  if((az>0.2)||(az<(-0.2))){
    vz = vz + az*time;
    sz = sz + vz*time;
  }
  
  /*
  sx = sx+vx*time+ax/2*pow(time,2);
  sy = sy+vy*time+ay/2*pow(time,2);
  sz = sz+vz*time+az/2*pow(time,2);
  */
  
  /*------------------------------------------------------------
  angleXg = ax/9.81;
  angleYg = ay/9.81;
  angleZg = az/9.81;
  
  //ITG
  readFrom(ITG, gyroAddress, TO_READ_GYRO, gyroValues);
  gX = (((int)gyroValues[0])<< 8) | gyroValues[1];
  gY = (((int)gyroValues[2])<< 8) | gyroValues[3]; 
  gZ = (((int)gyroValues[4])<< 8) | gyroValues[5];
  delay(15); 

  gyroX = gX/14.375+5; // integrieren -> Winkeländerung
  gyroY = gY/14.375;
  gyroZ = gZ/14.375;
  
  angleXaccel = 90-(acos(ay/sqrt(pow(ay,2)+pow(az,2))))*360/2/M_PI;
  angleYaccel = 90-(acos(ax/sqrt(pow(ax,2)+pow(az,2))))*360/2/M_PI;
  angleZaccel = 90-(acos(ay/sqrt(pow(ay,2)+pow(ax,2))))*360/2/M_PI;
  
  angleX = (0.98)*(angleX+gyroX*time)+(0.02*angleXaccel);
  angleY = (0.98)*(angleY+gyroY*time)+(0.02*angleYaccel);
  angleZ = (0.98)*(angleZ+gyroZ*time)+(0.02*angleZaccel);
  -------------------------------------------------------------
  */
  
  //Output to serial monitor
  
  /*
  //ADXL
  Serial.print("angleXaccel:");
  Serial.println(angleXaccel);
  Serial.print("angleYaccel:");
  Serial.println(angleYaccel);
  Serial.print("angleZaccel:");
  Serial.println(angleZaccel);
  Serial.println(" ");
  */
  
  Serial.println("X: ");
  Serial.println(ax);
  Serial.println("Y: ");
  Serial.println(ay); 
  Serial.println("Z: ");
  Serial.println(az);  
  Serial.println(" "); 
    
  Serial.println("OrtX: ");
  Serial.println(sx);
  Serial.println("OrtY ");
  Serial.println(sy); 
  Serial.println("OrtZ: ");
  Serial.println(sz);  
  Serial.println(" "); 
 
  /*
  //ITG
  Serial.print("gyroX: ");
  Serial.println(gyroX);
  Serial.print("gyroY: ");
  Serial.println(gyroY);
  Serial.print("gyroZ: ");
  Serial.println(gyroZ); 
  Serial.println(" ");
  Serial.println(" "); 
  
  Serial.print("angleX:");
  Serial.println(angleX);
  Serial.print("angleY:");
  Serial.println(angleY);
  Serial.print("angleZ:");
  Serial.println(angleZ);
  Serial.println(" ");
  */
  
  delay(50);
  
  tn=millis()-ta;
  Serial.println(tn);
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

void sortArray(int array[], int num){
  int temp=0;
  
  for(int i=0; i<num; i++){ 
    for(int j=i+1; j<num; j++){
      if(array[i]>array[j]){
        temp=array[i];
        array[i]=array[j];
        array[j]=temp;
      }
    }
  }
}

