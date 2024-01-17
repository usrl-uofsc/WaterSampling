 /*
  This sketch is for testing and debugging a library of functions for a sensor based on the Arduino Nano BLE sense
*/

#include "Drone_sensor.h"
#include <ros.h>                  // ROS main library and data type libraries
#include <ros/time.h>
#include <std_msgs/Int16.h>       
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>


//#define DEBUG 1     // code scaffold for enabling/disabling debugging output on the serial monitor
#define ROS_TIMEOUT_DELAY 100

Drone_sensor sensor;        // set up sensor with 10 Hz report freq

Drone_sensor::ADC_channel<128> fluorescence(8, NRF_SAADC_INPUT_AIN2); // create fluorescence channel object with 128 result buffer, oversample = 8, on pin AIN2
Drone_sensor::ADC_channel<128> reference(8, NRF_SAADC_INPUT_AIN6); // reference channel object
Drone_sensor::ADC_channel<128> scattering(8, NRF_SAADC_INPUT_AIN5); // scattering channel object
Drone_sensor::ADC_channel<128> raman(8, NRF_SAADC_INPUT_AIN6); // raman channel
Drone_sensor::ADC_channel<128> pe(8, NRF_SAADC_INPUT_AIN5); // phycoerythrin channel

ros::NodeHandle nh;               // create node handle object for ROS

void messageCb(const std_msgs::Int16& mode) {       // Callback function for subscriber input on ROS
    sensor.setMode(mode.data);
}

ros::Subscriber<std_msgs::Int16> mode("sensor/mode", messageCb );    // Declare subscriber to messages on "mode" topic, trigger callback function

std_msgs::Float32 fluorMsg;                                     // Set up data type for fluorescence data message
std_msgs::Float32 refMsg;                                       // Set up data type for reference data message
std_msgs::Float32 scatMsg;                                       // Set up data type for scattering data message
std_msgs::Float32 ramanMsg;                                     // Set up data type for raman data message
std_msgs::Int16 setMsg;
std_msgs::Float32 temperature;                                // Set up data type for temp data message
std_msgs::Float32 humidity;                                   // Set up data type for humidity data message
std_msgs::String myErrorMsg;                                       // Set up error message
std_msgs::UInt32 calibrationMsg;				// Set up initiate calibration msg? WSchuler
std_msgs::Int16 diagnostic;					// Set up initiate diagnostic msg? WSchuler	

ros::Publisher fluorPub("sensor/fluor", &fluorMsg);             // Declare publisher of fluorescence data message on "sensor/fluor" topic
ros::Publisher refPub("sensor/ref", &refMsg);                   // Declare publisher of reference data message on "sensor/ref" topic
ros::Publisher scatPub("sensor/scat", &scatMsg);                   // Declare publisher of scattering data message on "sensor/scat" topic
ros::Publisher ramanPub("sensor/raman", &ramanMsg);                   // Declare publisher of scattering data message on "sensor/scat" topic // Is this supposed to be "sensor/raman" instead of "sensor/scat"? WSchuler
ros::Publisher setPub("sensor/settings", &setMsg);		     // Declare publisher of set data message on "set" topic? WSchuler
ros::Publisher tempPub("sensor/temp", &temperature);                 // Declare publisher of temp data message on "temp" topic
ros::Publisher humPub("sensor/humidity", &humidity);                 // Declare publisher of humidity data message on "humidity" topic
ros::Publisher errorPub("sensor/error", &myErrorMsg);                     // Set up publisher of error topic
ros::Publisher calibrationPub("sensor/calibration", &calibrationMsg);	// Declare publisher of calibration data message on "calibration" topic? WSchuler
ros::Publisher diagnosticPub("sensor/diagnostic", &diagnostic);		// Declare publisher of diagnostic data message on "diagnostic" topic? WSchuler

unsigned long rosTimeout = ROS_TIMEOUT_DELAY;				//returns nonegative data for ROS delay? WSchuler
unsigned long tempTimeout = 1000;					//returns nonegative data for temperature timeout? WSchuler

//------------------------------------------------------------------------------
// Call back for file timestamps.  Only called for file create and sync().
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  DateTime now = sensor.rtc.now();

  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(now.year(), now.month(), now.day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(now.hour(), now.minute(), now.second());

  // Return low time bits in units of 10 ms, 0 <= ms10 <= 199.
  *ms10 = now.second() & 1 ? 100 : 0;
}

void setup() {
#ifndef DEBUG
    nh.initNode();                 // Initialize node handle
    nh.advertise(fluorPub);        // Start advertising/publishing on "fluor_data" topic
    nh.advertise(refPub);          // Start advertising/publishing on "ref_data" topic
    nh.advertise(scatPub);          // Start advertising/publishing on "ref_data" topic
    nh.advertise(ramanPub);	   // Start advertising/publishing on "sensor/scat" topic // Is this supposed to be "sensor/raman" instead of "sensor/scat"? WSchuler
    nh.advertise(setPub);	   // Start advertising/publishing on "set" topic WSchuler
    nh.advertise(tempPub);         // Start advertising/publishing on "temp" topic
    nh.advertise(humPub);          // Start advertising/publishing on "humidity" topic
    nh.advertise(errorPub);        // Advertise error topic
    nh.advertise(calibrationPub);  // Start advertising/publishing on "calibration' topic WSchuler
    nh.advertise(diagnosticPub);  // Start advertising/publishing on "diagnotic" topic WSchuler
    nh.subscribe(mode);            // Initialize subscriber to messages on "record" topic 
#endif // !DEBUG

  // Set file timestamp callback
  FsDateTime::setCallback(dateTime);

  #ifdef DEBUG
    Serial.begin(115200);    // use Serial output for debugging
    while (!Serial) {}
  #endif // DEBUG

	if (sensor.init()) { // initialize sensor
    sensor.setMode(2);
    #ifdef DEBUG
// print a msg based on whether sensor initiates WSchuler
        Serial.println("Sensor.init success");
      }
      else {
        Serial.println("Sensor.init failed");
    #endif // DEBUG
  }
}


void loop() {
    switch (sensor.getMode()) {
	// Idle Mode    
    case 0: {
 	if (sensor.isActive()) {
            sensor.shutdown();
      #ifdef DEBUG
	Serial.println(F("Shutting down"));
      #endif // DEBUG
    }
    #ifndef DEBUG 
    if (sensor._errorCase) {
	myErrorMsg.data = sensor.readErrors();
	errorPub.publish(&myErrorMsg);
    }
    #endif // DEBUG
    
    if (millis() > rosTimeout) { // this case indicates an error and in respone updates the ROS WSchuler
      #ifdef DEBUG
        if (sensor._errorCase != 0) {
          Serial.println(sensor._errorCase);
          Serial.println(sensor.readErrors());
          Serial.println(sensor._errorCase);
        }
      #endif // DEBUG
      #ifndef DEBUG
        nh.spinOnce();		// update ROS WSchuler
      #endif // !DEBUG
      rosTimeout = millis() + ROS_TIMEOUT_DELAY;
    } 

    if (millis() > tempTimeout) {    // once per second, the sensor will read new temp and humidity values and store them to the SD card
      sensor.getHTS();
      temperature.data = sensor.HTSdata.temperature;
      humidity.data = sensor.HTSdata.humidity;
      #ifndef DEBUG
        tempPub.publish(&temperature);
        humPub.publish(&humidity);
      #endif // !DEBUG
      #ifdef DEBUG 
        Serial.print("Temp: ");
        Serial.println(sensor.HTSdata.temperature);
        Serial.print("Humidity: ");
        Serial.println(sensor.HTSdata.humidity);
      #endif // DEBUG
      tempTimeout = millis() + 1000;
    }
    break;
  }// end case 0
	    
  // Record Mode (data)
  case 1: { // Might be helpful to readers to explain what the case is? WSchuler
    Drone_sensor::ADC<3072> adc;
    adc.enableChannel(&fluorescence);
    adc.enableChannel(&reference);
    adc.enableChannel(&scattering);
    sensor.initADC(&adc);

    #ifdef DEBUG
      Serial.print(adc._numChannels);
      Serial.println(F(" channels enabled"));
      Serial.println(F("Creating new file"));
    #endif // DEBUG

    sensor.newFile();
    sensor.start(0);                // start the sensor with the laser off to collect the background
    uint8_t sampleCounter = 0;      // the sample counter will rollover every 256 samples

    while (sensor.getMode() == 1) {
      while (!sensor.dataReady()) {}      // wait for data buffer to fill
      sensor.laser(sampleCounter != 255); // turn laser off every 256th sample (on otherwise)

      #ifdef DEBUG
        Serial.println(F("Getting data"));
      #endif // DEBUG
      fluorMsg.data = adc.getData(&fluorescence);
      refMsg.data = adc.getData(&reference);
      scatMsg.data = adc.getData(&scattering);

      if(adc.OOR_flag){ 
        setMsg.data = (100*fluorescence.config.gain) + (10*reference.config.gain) + (scattering.config.gain);
        #ifndef DEBUG
          setPub.publish(&setMsg);
        #endif // !DEBUG
        #ifdef DEBUG
          Serial.print(F("Autogain: "));
          Serial.println(setMsg.data);
        #endif // DEBUG
        adc.OOR_flag = false; // clear flag
      }

      sampleCounter++;                // increment sample counter
      sensor.start();                 // start next collection
      sensor.writeFile(fluorescence); // write data to SD card
      sensor.writeFile(reference);
      sensor.writeFile(scattering);
      
      #ifndef DEBUG
        fluorPub.publish(&fluorMsg);    // publish fluor on the "fluor_data" ROS topic
        refPub.publish(&refMsg);        // publish ref on the "ref_data" ROS topic
        scatPub.publish(&scatMsg);      // publish scat data
        nh.spinOnce();                  // update ROS
      #endif // !DEBUG

      #ifdef DEBUG 
        Serial.print("Data:\t");
        Serial.print(fluorMsg.data);
        Serial.print("\t");
        Serial.print(refMsg.data);
        Serial.print("\t");
        Serial.println(scatMsg.data);
        if (Serial.available()) {
            sensor.setMode(Serial.parseInt());
        }
      #endif // DEBUG
    }
    sensor.shutdown();
    break;
  } // end case 1
	    
  // Calibrate (cali)
  case 2: { // Might be helpful to readers to explain what the case is? WSchuler
    #ifdef DEBUG
      Serial.println("Running calibration");
    #endif // DEBUG
    if (!sensor.newFolder()) {
        break;
    }
    if (!sensor.newFile()) {
        break;
    }

    ros::Time ROStimestamp = nh.now();
    DateTime startupTimeROS = DateTime(ROStimestamp.sec);
    DateTime startupTimeRTC = sensor.rtc.now();

    Drone_sensor::ADC<2048> adc;
    adc.enableChannel(&fluorescence);
    adc.enableChannel(&reference);
    sensor.initADC(&adc);

    float bkgdFluor = 0;
    float laserIntensity = 0;
    float blank[100] = { 0 };
    float sse = 0;
    float blankNoise = 0;

    for (int i = 0; i < 100; i++) {
        sensor.start();
        while (!sensor.dataReady()) {}
        if(adc.OOR_flag){
          setMsg.data = (100*fluorescence.config.gain) + (10*reference.config.gain) + (scattering.config.gain);
          #ifndef DEBUG
            setPub.publish(&setMsg);
          #endif // !DEBUG
          #ifdef DEBUG
            Serial.print(F("Autogain: "));
            Serial.println(setMsg.data);
          #endif // DEBUG
          adc.OOR_flag = false; // clear flag
        } 
        sensor.laser(false);
        blank[i] = adc.getData(&fluorescence);
        laserIntensity += adc.getData(&reference);

        sensor.start();
        while (!sensor.dataReady()) {}
        if(adc.OOR_flag){
          setMsg.data = (100*fluorescence.config.gain) + (10*reference.config.gain) + (scattering.config.gain);
          #ifndef DEBUG
            setPub.publish(&setMsg);
          #endif // !DEBUG
          #ifdef DEBUG
            Serial.print(F("Autogain: "));
            Serial.println(setMsg.data);
          #endif // DEBUG
          adc.OOR_flag = false; // clear flag
        } 
        sensor.laser(true);
        blank[i] -= adc.getData(&fluorescence);
        bkgdFluor += blank[i];
        laserIntensity -= adc.getData(&reference);
        #ifndef DEBUG
          if (i % 10 == 0) {
              nh.spinOnce();
          }
        #endif // !DEBUG
    }

    bkgdFluor = bkgdFluor / 100.0;
    laserIntensity = laserIntensity / 100.0;

    for (int i = 0; i < 100; i++) {
        sse += sq(blank[i] - bkgdFluor);
    }
    blankNoise = sqrt(sse / 100.0);

    sensor.file.println(F("Startup and Calibration"));
    sensor.file.print(F("RTC timestamp: "));
    sensor.file.println(startupTimeRTC.timestamp(DateTime::TIMESTAMP_FULL));
    sensor.file.print(F("ROS timestamp: "));
    sensor.file.println(startupTimeROS.timestamp(DateTime::TIMESTAMP_FULL));
    sensor.file.print(F("Sensor internal temperature: "));
    sensor.file.print(HTS.readTemperature());
    sensor.file.println(F(" C"));
    sensor.file.print(F("Sensor internal humidity: "));
    sensor.file.println(HTS.readHumidity());
    sensor.file.print(F("Laser intensity (uV): "));
    sensor.file.println(laserIntensity);
    sensor.file.print(F("Background fluorescence (uV): "));
    sensor.file.println(bkgdFluor);
    sensor.file.print(F("Fluorescence detector blank noise (uV): "));
    sensor.file.println(blankNoise);
    //sensor.file.print(F("Ambient light: "));
    //sensor.file.println(ambientLight);
    //sensor.file.print(F("Calibration: "));
    //sensor.file.print(calibration);
    //sensor.file.println(F("(equivalent to ?? ucg/L chl a standard)"));
    //sensor.file.print(F("Diagnostic code: "));
    //sensor.file.println(diagnosticCode);

    sensor.shutdown();
    sensor.setMode(0);
    break;
  } // end case 2
	    
  // Laser On
  case 3: {
    sensor.laser(true);
    sensor.setMode(0);
    break;
  }
  // Laser Off
  case 4: {
    sensor.laser(false);
    sensor.setMode(0);
    break; 
  } 
// disable modes 14 and 15
	    
// Record Mode for remote sensor (data)
case 10: {
    Drone_sensor::ADC<3072> adc;
    adc.enableChannel(&fluorescence);
    adc.enableChannel(&raman);
    sensor.initADC(&adc);

    #ifdef DEBUG
      Serial.print(adc._numChannels);
      Serial.println(F(" channels enabled"));
      Serial.println(F("Creating new file"));
    #endif // DEBUG

    sensor.newFile();
    sensor.start(0);                // start the sensor with the laser off to collect the background
    uint8_t sampleCounter = 0;      // the sample counter will rollover every 256 samples

    while (sensor.getMode() == 10) {
      while (!sensor.dataReady()) {}      // wait for data buffer to fill
      sensor.laser(sampleCounter != 255); // turn laser off every 256th sample (on otherwise)

      #ifdef DEBUG
        Serial.println(F("Getting data"));
      #endif // DEBUG
      fluorMsg.data = adc.getData(&fluorescence);
      ramanMsg.data = adc.getData(&raman);

      if(adc.OOR_flag){
        setMsg.data = (100*fluorescence.config.gain) + (10*raman.config.gain);
        #ifndef DEBUG
          setPub.publish(&setMsg);
        #endif // !DEBUG
        #ifdef DEBUG
          Serial.print(F("Autogain: "));
          Serial.println(setMsg.data);
        #endif // DEBUG
        adc.OOR_flag = false; // clear flag
      }

      sampleCounter++;                // increment sample counter
      sensor.start();                 // start next collection
      sensor.writeFile(fluorescence); // write data to SD card
      sensor.writeFile(raman);
      
      #ifndef DEBUG
        fluorPub.publish(&fluorMsg);    // publish fluor on the "fluor_data" ROS topic
        ramanPub.publish(&ramanMsg);        // publish ref on the "ref_data" ROS topic
        nh.spinOnce();                  // update ROS
      #endif // !DEBUG

      #ifdef DEBUG
        Serial.print("Data:\t");
        Serial.print(fluorMsg.data);
        Serial.print("\t");
        Serial.print(ramanMsg.data);
        if (Serial.available()) {
            sensor.setMode(Serial.parseInt());
        }
      #endif // DEBUG
    }
    sensor.shutdown();
    break;
  } // end case 10
  default: {
    sensor.setMode(0);
    break;
  }
} // end switch
	
    #ifdef DEBUG
    if (Serial.available()) {
        sensor.setMode(Serial.parseInt());
    }
    #endif // DEBUG
    #ifndef DEBUG
    if (millis() > rosTimeout) {
	nh.spinOnce();
	rosTimeout = millis() + ROS_TIMEOUT_DELAY;
    }
    #endif // !DEBUG
} // end loop

/* callback for microphone */
void onPDMdata() {
  // Query the number of available bytes
  sensor.PDMdata.bytesAvailable = PDM.available();
  PDM.read(sensor.PDMdata.sampleBuffer, sensor.PDMdata.bytesAvailable);
  sensor.PDMdata.flag = true;
}
