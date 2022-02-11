/***************************************************************************
* Example sketch for the ICM20948_WE library
*
* This sketch can be used to determine the (non-auto) offsets for the ICM20948. 
* It does not use the internal offset registers.
* 
* For the gyroscope offsets just use the raw values you obtain when the module is 
* not moved.
* 
* For the accelerometer offsets turn the ICM20948 slowly(!) in all directions and find the 
* minimum and maximum raw values for all axes.
* 
* Insert the values in the corresponding setxxxOffsets() functions. 
* 
* Further information can be found on:
*
* https://wolles-elektronikkiste.de/icm-20948-9-achsensensor-teil-i (German)
* https://wolles-elektronikkiste.de/en/icm-20948-9-axis-sensor-part-i (English)
* 
***************************************************************************/

#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68
char Incoming_value = 0;    

/* There are several ways to create your ICM20948 object:
 * ICM20948_WE myIMU = ICM20948_WE()              -> uses Wire / I2C Address = 0x68
 * ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR) -> uses Wire / ICM20948_ADDR
 * ICM20948_WE myIMU = ICM20948_WE(&wire2)        -> uses the TwoWire object wire2 / ICM20948_ADDR
 * ICM20948_WE myIMU = ICM20948_WE(&wire2, ICM20948_ADDR) -> all together
 * Successfully tested with two I2C busses on an ESP32
 */
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
// Visual feedback inputs and pinouts 
int red_light_pin= 11;
int green_light_pin = 10;
int blue_light_pin = 9;
 
int user_angle;// From renes calculations  
void setup() {
  Wire.begin();
  Serial.begin(115200);
  while(!Serial) {}
  
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  else{
    Serial.println("ICM20948 is connected");
  }
  
 if(!myIMU.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }
  /*  This is a method to calibrate. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  The parameters are floats. 
   *  The calibration changes the slope / ratio of raw accleration vs g. The zero point is 
   *  set as (min + max)/2.
   */
  //myIMU.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16520.0, 16690.0);
    
  /*  The gyroscope data is not zero, even if you don't move the ICM20948. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffset or setGyrOffsets, not both.
   */
  //myIMU.setGyrOffsets(-115.0, 130.0, 105.0);
  
    
  /*  ICM20948_ACC_RANGE_2G      2 g   (default)
   *  ICM20948_ACC_RANGE_4G      4 g
   *  ICM20948_ACC_RANGE_8G      8 g   
   *  ICM20948_ACC_RANGE_16G    16 g
   */

// Automatic setting 
  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!"); 
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G); // highest res for calibration
  
  /*  Choose a level for the Digital Low Pass Filter or switch it off.  
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              246.0               1125/(1+ASRD) 
   *    1              246.0               1125/(1+ASRD)
   *    2              111.4               1125/(1+ASRD)
   *    3               50.4               1125/(1+ASRD)
   *    4               23.9               1125/(1+ASRD)
   *    5               11.5               1125/(1+ASRD)
   *    6                5.7               1125/(1+ASRD) 
   *    7              473.0               1125/(1+ASRD)
   *    OFF           1209.0               4500
   *    
   *    ASRD = Accelerometer Sample Rate Divider (0...4095)
   *    You achieve lowest noise using level 6  
   */
  myIMU.setAccDLPF(ICM20948_DLPF_6); // lowest noise for calibration   
  
  /*  ICM20948_GYRO_RANGE_250       250 degrees per second (default)
   *  ICM20948_GYRO_RANGE_500       500 degrees per second
   *  ICM20948_GYRO_RANGE_1000     1000 degrees per second
   *  ICM20948_GYRO_RANGE_2000     2000 degrees per second
   */
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250); //highest resolution for calibration
  
  /*  Choose a level for the Digital Low Pass Filter or switch it off. 
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              196.6               1125/(1+GSRD) 
   *    1              151.8               1125/(1+GSRD)
   *    2              119.5               1125/(1+GSRD)
   *    3               51.2               1125/(1+GSRD)
   *    4               23.9               1125/(1+GSRD)
   *    5               11.6               1125/(1+GSRD)
   *    6                5.7               1125/(1+GSRD) 
   *    7              361.4               1125/(1+GSRD)
   *    OFF          12106.0               9000
   *    
   *    GSRD = Gyroscope Sample Rate Divider (0...255)
   *    You achieve lowest noise using level 6  
   */
  myIMU.setGyrDLPF(ICM20948_DLPF_6); // lowest noise for calibration
  
  /*  Choose a level for the Digital Low Pass Filter. 
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  DLPF          Bandwidth [Hz]      Output Rate [Hz]
   *    0             7932.0                    9
   *    1              217.9                 1125
   *    2              123.5                 1125
   *    3               65.9                 1125
   *    4               34.1                 1125
   *    5               17.3                 1125
   *    6                8.8                 1125
   *    7             7932.0                    9
   *                 
   *    
   *    GSRD = Gyroscope Sample Rate Divider (0...255)
   *    You achieve lowest noise using level 6  
   */
  myIMU.setTempDLPF(ICM20948_DLPF_6); // lowest noise for calibration
  myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
  myIMU.setAccSampleRateDivider(10);

  // VISUAL FEEDBACK & LED 
  
  Serial.begin(9600);         //Sets the data rate in bits per second (baud) for serial data transmission
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
}

void loop() {
  myIMU.readSensor();
  xyzFloat accRaw;
  xyzFloat gyrRaw;
  xyzFloat corrAccRaw;
  xyzFloat corrGyrRaw;
  xyzFloat magValue = myIMU.getMagValues(); // returns magnetic flux density [µT]

  accRaw = myIMU.getAccRawValues();
  gyrRaw = myIMU.getGyrRawValues();
  corrAccRaw = myIMU.getCorrectedAccRawValues();
  corrGyrRaw = myIMU.getCorrectedGyrRawValues();
  xyzFloat gVal = myIMU.getGValues();
  
  Serial.println("Acceleration raw values without offset:");
  Serial.print(accRaw.x);
  Serial.print("   ");
  Serial.print(accRaw.y);
  Serial.print("   ");
  Serial.println(accRaw.z);

  Serial.println("Gyroscope raw values without offset:");
  Serial.print(gyrRaw.x);
  Serial.print("   ");
  Serial.print(gyrRaw.y);
  Serial.print("   ");
  Serial.println(gyrRaw.z);

  Serial.println("Acceleration raw values with offset:");
  Serial.print(corrAccRaw.x);
  Serial.print("   ");
  Serial.print(corrAccRaw.y);
  Serial.print("   ");
  Serial.println(corrAccRaw.z);

  Serial.println("Gyroscope raw values with offset:");
  Serial.print(corrGyrRaw.x);
  Serial.print("   ");
  Serial.print(corrGyrRaw.y);
  Serial.print("   ");
  Serial.println(corrGyrRaw.z);

  Serial.println("g-values, based on corrected raws (x,y,z):");
  Serial.print(gVal.x);
  Serial.print("   ");
  Serial.print(gVal.y);
  Serial.print("   ");
  Serial.println(gVal.z);

  Serial.println("********************************************");
  // corrected_x = (sensor_x - offset_x) * scale_x
  magValue.x= (magValue.x-(-3.585))*1.13651;
  magValue.y= (magValue.y-(2.32))*0.9068;
  magValue.z= (magValue.z-(-12.63))*0.99027;
 
  Serial.println(" Corrected Magnetometer Data in µTesla: ");
  Serial.print(magValue.x);
  Serial.print("   ");
  Serial.print(magValue.y);
  Serial.print("   ");
  Serial.println(magValue.z);
  Serial.println("********************************************");

  delay(500); // may need to move 
//----------------------------------------------------------------------
// LED CODE 
// using 0,2,4 as switch cases 
 // switch -- maybe use 

 if(Serial.available() > 0)  
  {
    Incoming_value = Serial.read();      //Read the incoming data and store it into variable Incoming_value
    Serial.print(Incoming_value);        //Print Value of Incoming_value in Serial monitor
    Serial.print("\n");        //New line 
    // week 0 -2-------------------------------------------------------------------
    if(Incoming_value == '0') {
    
      
        RGB_color(0, 255, 0); // Green
        delay(2000); //
      }   
        
    else if(Incoming_value == '1') {
         RGB_color(255, 0, 0); // Red }
         delay(2000);
        }
     
      }

   

// END OF VISUAL FEEDBACK-------------------------------------------------------------------


}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}

// RENE angle calc
