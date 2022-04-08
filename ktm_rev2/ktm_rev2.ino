/* BME 70A/B CAPSTONE DESIGN
 *  Firmware for MKR WiFi 1010 for the OG02 Motion Tracking and Analysis Wearable.
 *  Author: Rene Gilliland-Rocque
 *  
 *  This file incorporates a few featuers
 *    - Bluetooth
 *        The board has a wifi module capable BLE 4.2. This is used to send data to the web application 
 *        located at https://kneetome.herokuapp.com/. Definitely use chrome for this.
 *        
 *        The board is configured to update the data in the bluetooth characteristic at roughly 50Hz.
 *        
 *        There is functionality for having the web application send start and stop conditions but these were
 *        not implemented in time on the server side so the functionality was removed from this project. 
 *        
 *    - I2C comms with an ICM20948 
 *        The serial comms were done through a library sourced by Student A. The library has a strange method of 
 *        abstracting nearly all of the actual data transfer and registers accessed away from the developer and
 *        requires that the method "readSensor()" be called before any data can be stored in volatile memory. 
 *        
 *        There are prebuilt orientation calculations in the library but they are very buggy and can be extremely unreliable. 
 *        
 *        The sensor is able to output 9 DOF: xyz acceleration, xyz angular velocity, xyz magnetic field strength.
 *        
 *    - Sensor Orientation
 *        A complimentary angle filter continuously calculates the device orientation.
 *        The filter is tuned to use the gyroscope data short term and the accelerometer data long term. 
 *        The inherent weaknesses for both sensed metrics can be overcome by using them in this configuration (gyro drift, slow accel).
 *        
 *        The C++ match library is required to handle the floating point calculations although the user is presented with a rounded integer.
 *        
 *    - 3 Common Cathode LEDs
 *        The board houses 3, 60ohm 1/4W resistors connected to pins 0, 13 and 14. These can be connected to the pins of the appropriate LED
 *        to allow the device to drive the LEDs (GIVEN THE CURRENT IS WIHTIN SPEC FOR THE MCU). In this project we used a 10mm CC RGB LED from
 *        Creatron that worked relatively well. It needs difusing to spread the light better but it is very large.  
 *        
 *    - Buzzer
 *        
 */




#include <Wire.h>
#include <ICM20948_WE.h>
#include <ArduinoBLE.h>
#include <math.h>

// Sensor I2C address (not sure why this isn't part of the standard constructor in the library...)
#define ICM20948_ADDR 0x68

// UUIDs for BLE characteristics
#define POSITION_QUALITY1_UUID "2A69"
#define POSITION_QUALITY2_UUID "2A70"
#define POSITION_QUALITY3_UUID "2A71"
#define POSITION_QUALITY4_UUID "2A72"

// feedback constants
#define RED_LED 13
#define GREEN_LED 14
#define BLUE_LED 0
#define BUZZER 1
#define CALIB_DELAY 3000

//create BLE service with the "user data" GATT name
BLEService user_service("181C");  

//BLE characteristics for each data point, xyz angles in integers, start stop flags and led flags
BLEUnsignedIntCharacteristic angle_x(POSITION_QUALITY1_UUID, BLERead|BLEWrite|BLENotify);
BLEUnsignedIntCharacteristic feedback(POSITION_QUALITY2_UUID, BLERead|BLEWrite|BLENotify);
BLEBoolCharacteristic start_flag(POSITION_QUALITY3_UUID, BLERead|BLEWrite|BLENotify);
BLEBoolCharacteristic calibration_flag(POSITION_QUALITY4_UUID, BLERead|BLEWrite|BLENotify);

// sensor object setup
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// vars
int USER_ANGLE = 0;
int ANGLE_TH = 90;
int ANGLE_REF = 0;
int ANGLE_OLD = 0;
int ANGLE_NEW = 0;
int TONE_FREQ_1 = 200;
int TONE_FREQ_2 = 400;
int TONE_FREQ_3 = 600;
int TONE_FREQ_4 = 5000;
int local_feedback = 0;
int dummy_week = 14;
int TIME_OLD = 0;
int TIME_NEW = 0;
int TIME_DELTA = 0;
float NEW_ANGLE = 0;
  
// local flags
bool local_start_flag = false;
bool local_calib_flag = false;
bool tone_flag = false;


/* ble_init(BLEService bleserv, const char* namestr)
 *  this function initialized the bluetooth module on the board. it is passed 2 parameters
 *  
 *  bleserv ->  Object of type BLEService from the ArduinoBLE library.
 *  namestr ->  User defined name for the service. This will change the name of the bluetooth device
 *              as it shows up on the host (central) device.
 */
inline void ble_init(BLEService bleserv, const char* namestr){
  //init BLE, print error msg if it fails
  if(!BLE.begin()){ 
    Serial.println("BLE init failed"); 
    }

  //set the name for the device and advertise the service
  BLE.setLocalName(namestr);
  BLE.setAdvertisedService(bleserv);

  //add all characteristics to the service and add the service to the current BLE instance
  bleserv.addCharacteristic(angle_x);
  bleserv.addCharacteristic(feedback);
  bleserv.addCharacteristic(start_flag);
  bleserv.addCharacteristic(calibration_flag);
  
  BLE.addService(bleserv);

  //set inital values for all characteristics; inital state is off
  angle_x.writeValue(0);
  feedback.writeValue(0);
  start_flag.writeValue(false);
  calibration_flag.writeValue(false);

  //advertize the service
  BLE.advertise();
}

/* LED update function
 *  the LEDs are setup with a common cathode, thus
 *  the pins are sourcing the current for the light control. 
 *  a HIGH/TRUE value will result in the LED emiting light and
 *  a LOW/FALSE value will result in the LED not emiting light
 */
inline void led_update( int LED1, 
                        bool LED1_state, 
                        int LED2,
                        bool LED2_state,
                        int LED3,
                        bool LED3_state ) {
                          
  digitalWrite(LED1, LED1_state);
  digitalWrite(LED2, LED2_state);
  digitalWrite(LED3, LED3_state);
}

void setup() {
  /*LED init*/
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  
  /*Serial init*/
  Wire.begin();         //setup I2C peripherals
  Serial.begin(115200); //setup serial connection for debugging
  delay(1500);         //2s delay to give people setup time
  
  /*sensor init*/
      if(!myIMU.init()){
        Serial.println("ICM20948 does not respond");
      }
      else{
        Serial.println("ICM20948 is connected");
      }
    
      /* init magnetometer, print status*/
      if(!myIMU.initMagnetometer()){
        Serial.println("Magnetometer does not respond");
      }
      else{
        Serial.println("Magnetometer is connected");
      }
      delay(1000);
    
      /*  This will be moved further into the code to change on a status flag.
       *  The status flag will be set once the start button is pressed by the 
       *  user and the calibration has completed
       */
       //myIMU.autoOffsets();
    
       /*set accelerometer and gyroscope ranges and filters*/
       myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
       myIMU.setAccDLPF(ICM20948_DLPF_6);
       myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
       myIMU.setGyrDLPF(ICM20948_DLPF_6);
    
       /* set temperature low-pass filter */
       myIMU.setTempDLPF(ICM20948_DLPF_6);
    
       /* set magnetometer mode */
       myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
    
       /*set accelerometer sample rate divider */
       myIMU.setAccSampleRateDivider(10);
  
  /*BLE init*/
  ble_init(user_service, "knee to me");
}

void loop() {

  /*variables!*/
  
  //this is how data was retrieved in the example project...not sure why each variable is being redfined each loop... but it works 
  myIMU.readSensor();

  xyzFloat accRaw = myIMU.getAccRawValues();
  xyzFloat gyrRaw = myIMU.getGyrRawValues();
  xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();
  xyzFloat corrGyrRaw = myIMU.getCorrectedGyrRawValues();
  TIME_NEW = (int) millis();
  TIME_DELTA = TIME_NEW - TIME_OLD;

  //manually calc the angle, complimentary angle filter
  float test_pitch = atan2f(corrAccRaw.x, corrAccRaw.z) * (180.0 / 3.141593);
  NEW_ANGLE = 0.98 * (NEW_ANGLE + (corrGyrRaw.x * TIME_DELTA)) + 0.02 * test_pitch;

  //vars for debugging via serial cmds
  char inbyte = 0;
  String xstr = "angle (degree): ";

  /* CALIBRATION CODE */
  
  //local_calib_flag needs to be set for data updates
  if(!local_calib_flag && ANGLE_REF == 0){
     //play a tone to signify the device is on and to move to flexion. wait 3 seconds then calibrate the reference position
    tone(BUZZER, TONE_FREQ_1);
    delay(750);
    noTone(BUZZER);
    delay(CALIB_DELAY);


    myIMU.readSensor();

    accRaw = myIMU.getAccRawValues();
    gyrRaw = myIMU.getGyrRawValues();
    corrAccRaw = myIMU.getCorrectedAccRawValues();
    corrGyrRaw = myIMU.getCorrectedGyrRawValues();
  
    //manually calc the angle, no need for gyro as the leg is stationary and the user has 3 seconds to hold the position steady.
    test_pitch = atan2f(corrAccRaw.x, corrAccRaw.z) * (180.0 / 3.141593);
    
  
    ANGLE_REF = test_pitch;
    Serial.print("ANGLE_REF: "  ); Serial.println(test_pitch);
    tone(BUZZER, TONE_FREQ_1);
    delay(200);
    tone(BUZZER, TONE_FREQ_2);
    delay(200);
    tone(BUZZER, TONE_FREQ_3);
    delay(200);
    noTone(BUZZER);
    local_calib_flag = true;
  }
  
  
  //contect to the central user device
  BLEDevice central = BLE.central();
  
        /* POLL FOR USER INPUTS AND UPDATE LOCAL FLAGS */
      
        //check if user has pressed start reccording and update local flag
        if(start_flag.written()){ local_start_flag = start_flag.value();  }
        //Serial.print("start flag: ");
        //Serial.println(local_start_flag);

        //check for calibration input from user
        if(calibration_flag.written()){ local_calib_flag = calibration_flag.value(); }

        //check for inital send of feedback week.
        if(feedback.written()){ local_feedback = feedback.value(); }
        
        //integer sent from the server for the current rehab week. 90deg for weeks 0 and 1, 130deg for weeks 2 - 5, 140deg for 6+.
        if(local_feedback < 2)      {ANGLE_TH = 90;}
        else if(local_feedback < 6) {ANGLE_TH = 130;}
        else                        {ANGLE_TH = 140;}
        //Serial.print("feedback week: ");
        //Serial.println(local_feedback);

        
        /*DATA UPDATE AND USER ANGLE CALC*/
        //angle displacement, current angle minus ref gives displacement and total ROM
        USER_ANGLE = NEW_ANGLE - ANGLE_REF;

        //fill a string with the data for debugging
        xstr += String(USER_ANGLE);
        xstr += ("    int conversion: ");
        xstr += String((int)USER_ANGLE);

        //update bluetooth characteristic and output string to USB for monitoring
        angle_x.writeValue(USER_ANGLE);
        Serial.println(xstr);
        
        /* FEEDBACK CODE 
         *  
         *  The LED is green when the user is at or above the threshold, and red otherwise.
         *  The buzzer works in a similar fashion where there is no tone as the use approaches the threshold
         *  and sounds the tone when the user hits their threshold.
         */
        if(USER_ANGLE < ANGLE_TH){ 
          led_update( RED_LED, true, 
                      GREEN_LED, false, 
                      BLUE_LED, false);
                      
          noTone(BUZZER); 
        } 
        else { 
          led_update( RED_LED, false, 
                      GREEN_LED, true, 
                      BLUE_LED, false); 
                      
          tone(BUZZER, TONE_FREQ_4);
        }

      // update the old time variable and introduce a small delay to limit the sampling rate to 50 Hz (main loop takes about 2ms).
      TIME_OLD = TIME_NEW;
      delay(18);
}
