/*
 * SYSTEM SPEED TEST
 * 
 * This code tests the speed at which the system can run without hard limiting the performance. 
 * The delay between subsequent loops is removed and a digital pin is toggled from low to high to show the start of the main loop.
 */

#include <Wire.h>
#include <ICM20948_WE.h>
#include <ArduinoBLE.h>
#include <math.h>

// Sensor I2C address in hexadecimal format
#define ICM20948_ADDR 0x68

// UUIDs for BLE characteristics
#define POSITION_QUALITY1_UUID "2A69"
#define POSITION_QUALITY2_UUID "2A70"
#define POSITION_QUALITY3_UUID "2A71"
#define POSITION_QUALITY4_UUID "2A72"

// feedback stuff
#define RED_LED 13
#define GREEN_LED 14
#define BLUE_LED 0
#define BUZZER 1
#define CALIB_DELAY 3000

// pin for calc output
#define CALC_OUT_PIN 8

//create BLE service with the "user data" GATT name
BLEService user_service("181C");  

//BLE characteristics for each data point, xyz angles in floating point, start stop flags and led flags
BLEUnsignedIntCharacteristic angle_x(POSITION_QUALITY1_UUID, BLERead|BLEWrite|BLENotify);
BLEUnsignedIntCharacteristic feedback(POSITION_QUALITY2_UUID, BLERead|BLEWrite|BLENotify);
BLEBoolCharacteristic start_flag(POSITION_QUALITY3_UUID, BLERead|BLEWrite|BLENotify);
BLEBoolCharacteristic calibration_flag(POSITION_QUALITY4_UUID, BLERead|BLEWrite|BLENotify);

//myIMU object setup
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

//vars
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
  
//flags
bool local_start_flag = false;
bool local_calib_flag = false;
bool tone_flag = false;

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
  pinMode(CALC_OUT_PIN, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(CALC_OUT_PIN, LOW);
  
  /*Serial init*/
  Wire.begin();         //setup I2C peripherals
  Serial.begin(115200); //setup serial connection for debugging
  while(!Serial)  {}    // wait for serial instance
  delay(1500);          //2s delay to give people setup time
  
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

  // toggle digital pin LOW for the start of the application
  digitalWrite(CALC_OUT_PIN, HIGH);
  
  //incoming sensor buffers, each has an x, y and z member variable  
  myIMU.readSensor();

  xyzFloat accRaw = myIMU.getAccRawValues();
  xyzFloat gyrRaw = myIMU.getGyrRawValues();
  xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();
  xyzFloat corrGyrRaw = myIMU.getCorrectedGyrRawValues();

  //manually calc the angle
  float test_pitch = atan2f(corrAccRaw.x, corrAccRaw.z) * (180.0 / 3.141593);
  //Serial.print("test pitch: "); Serial.println(test_pitch);

  //vars for debugging via serial cmds
  String xstr = "angle (degree): ";
  
  /* operational code */ 

  //calibration code
  
  //local_calib_flag needs to be set for data updates
  if(!local_calib_flag && ANGLE_REF == 0){
     //play a tone to signify the device is on and to move to flexion. wait 3 seconds then calibrate
    tone(BUZZER, TONE_FREQ_1);
    delay(750);
    noTone(BUZZER);
    delay(CALIB_DELAY);


    myIMU.readSensor();

    accRaw = myIMU.getAccRawValues();
    gyrRaw = myIMU.getGyrRawValues();
    corrAccRaw = myIMU.getCorrectedAccRawValues();
    corrGyrRaw = myIMU.getCorrectedGyrRawValues();
  
    //manually calc the angle
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

  /* POLL FOR USER INPUTS AND UPDATE FLAGS */

  //check if user has pressed start reccording and update local flag
  if(start_flag.written()){local_start_flag = start_flag.value();}

  //check for calibration input from user
  if(calibration_flag.written()){local_calib_flag = calibration_flag.value();}

  //check for inital send of feedback week, this determines the ROM target; 90 for weeks 0 and 1, 130 for weeks 2 - 5, 140 for 6+
  if(feedback.written()){ local_feedback = feedback.value();}
  if(local_feedback < 2)      {ANGLE_TH = 90;}
  else if(local_feedback < 6) {ANGLE_TH = 130;}
  else                        {ANGLE_TH = 140;}

  
  /*DATA UPDATE AND ANGLE CALC*/
    //angle displacement, current angle minus ref gives displacement
    USER_ANGLE = test_pitch - ANGLE_REF;

    //fill a string with the data for debugging
    xstr += String(USER_ANGLE);

    //update bluetooth characteristic and output string to USB for monitoring
    angle_x.writeValue(USER_ANGLE);
    Serial.println(xstr);
  
  /* FEEDBACK CODE */ 
  if(USER_ANGLE < ANGLE_TH){ 
    led_update(RED_LED, true, GREEN_LED, false, BLUE_LED, false);
    noTone(BUZZER); 
  } 
  else { 
    led_update(RED_LED, false, GREEN_LED, true, BLUE_LED, false); 
    tone(BUZZER, TONE_FREQ_4);
  }

  // toggle digital pin LOW for end of mainloop
  digitalWrite(CALC_OUT_PIN, LOW);
//delay(18);
}
