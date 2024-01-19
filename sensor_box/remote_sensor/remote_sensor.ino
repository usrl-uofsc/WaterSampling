/*
  This code is for a drone-mounted remote chlorophyll fluorescence, phycoerythrin fluorescence, and water raman sensor.

  The circuit: Arduino Nano 33 BLE Sense
  MOSFET N-channel transistor for switching laser using pin 9;
  fluorescence channel on A0; phycoerythrin channel on A1; raman channel on A2;
  communication with Raspberry Pi via ROS on UART/USB;
  SD card logger on SPI (pins 10-13).

  Pin 9 is controlled by tasks and events through the PPI to turn the laser on and off.

  The laser is switched at 10 Hz with a 50% duty cycle (50ms on/50ms off). During each 50 ms on/off period,
  128 samples will be taken on each channel and stored in a data array (total sampling
  rate = 256 S/0.05 s = 5,120 S/s).

  Each sample is a sum of 8 ADC conversions (each 12-bit), resulting in a 15-bit value (manual oversamling).
  Each set of 128 samples is summed together for each channel to produce a 22-bit value.
  Every 100 ms, a result is calculated for each channel as the difference between the on sum and the off sum (22-bit values).
  The resulting values are published to ROS topics which are monitored by the Pi.
  All samples are stored to the SD card.

  The ADC settings are:
    - Vref = 0.6 V
    - Mode = Single-ended
    - Gain = automatically adjusted individually for each channel
    - Acq time = 5 us
 
   Report frequency: 10 Hz

  There are 4 modes that can be used to operate the sensor
    0 -- Idle
    1 -- Record (creates a new file and stores each measurement to SD card)
    2 -- Calibrate (runs startup tests and generates a calibration; creates new folder in which subsequent files are stored)
    3 -- Laser on
    4 -- Laser off
*/

#include "Drone_sensor.h"
#include <ros.h>                  // ROS main library and data type libraries
#include <ros/time.h>
#include <std_msgs/Int16.h>       
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>


#define DEBUG 1     // code scaffold for enabling/disabling debugging output on the serial monitor
#define ROS_TIMEOUT_DELAY 100

Drone_sensor sensor;        // set up sensor with 10 Hz report freq

Drone_sensor::ADC_channel<128> chla(8, NRF_SAADC_INPUT_AIN2); // create fluorescence channel object with 128 result buffer, oversample = 8, on Nano pin A0
Drone_sensor::ADC_channel<128> pe(8, NRF_SAADC_INPUT_AIN3); // phycoerythrin channel on Nano pin A1
Drone_sensor::ADC_channel<128> raman(8, NRF_SAADC_INPUT_AIN6); // raman channel on Nano pin A2

ros::NodeHandle nh;               // create node handle object for ROS

void messageCb(const std_msgs::Int16& mode) {       // Callback function for subscriber input on ROS
    sensor.setMode(mode.data);
}

ros::Subscriber<std_msgs::Int16> mode("sensor/mode", messageCb );    // Declare subscriber to messages on "mode" topic, trigger callback function

std_msgs::Float32 chlaMsg;                                     // Set up data type for chl a data message
std_msgs::Float32 peMsg;                                     // Set up data type for PE data message
std_msgs::Float32 ramanMsg;                                     // Set up data type for raman data message
std_msgs::Int16 setMsg;
std_msgs::String myErrorMsg;                                       // Set up error message

ros::Publisher chlaPub("sensor/chla", &chlaMsg);             // Declare publisher of chl a fluorescence data message on "sensor/chla" topic
ros::Publisher pePub("sensor/pe", &peMsg);                  // Declare publisher of pe fluorescence data message on "sensor/pe" topic
ros::Publisher ramanPub("sensor/raman", &ramanMsg);            // Declare publisher of scattering data message on "sensor/raman" topic
ros::Publisher setPub("sensor/settings", &setMsg);             // publisher for settings topic
ros::Publisher errorPub("sensor/error", &myErrorMsg);          // Set up publisher of error topic

unsigned long rosTimeout = ROS_TIMEOUT_DELAY;

//------------------------------------------------------------------------------
// Call back for file timestamps.  Only called for file create and sync().
// Note: DON'T MOVE THIS CODE! It is needed for file timestamps, but I don't
// understand how or why it works, but it needs to be exactly right here to work.
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
    nh.advertise(chlaPub);        // Start advertising/publishing on "fluor_data" topic
    nh.advertise(pePub);          
    nh.advertise(ramanPub);
    nh.advertise(setPub);
    nh.advertise(errorPub);        // Advertise error topic
    nh.subscribe(mode);            // Initialize subscriber to messages on "record" topic
#endif // !DEBUG

  // Set file timestamp callback - DON'T TOUCH!
  FsDateTime::setCallback(dateTime);

  #ifdef DEBUG
    Serial.begin(115200);    // use Serial output for debugging
    while (!Serial) {}
  #endif // DEBUG

	if (sensor.init()) { // initialize sensor
    sensor.setMode(2);
    #ifdef DEBUG
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
    // shutdown sensor
    if (sensor.isActive()) {
      sensor.shutdown();
      #ifdef DEBUG
        Serial.println(F("Shutting down"));
      #endif // DEBUG
    }

    // check for error messages
    #ifndef DEBUG
      if (sensor._errorCase) {
        myErrorMsg.data = sensor.readErrors();
        errorPub.publish(&myErrorMsg);
      }
    #endif // !DEBUG
  
    // update and send error messages
    if (millis() > rosTimeout) {
      #ifdef DEBUG
        if (sensor._errorCase != 0) {
          Serial.println(sensor._errorCase);
          Serial.println(sensor.readErrors());
          Serial.println(sensor._errorCase);
        }
      #endif // DEBUG
      #ifndef DEBUG
        nh.spinOnce();
      #endif // !DEBUG
      rosTimeout = millis() + ROS_TIMEOUT_DELAY;
    } 
    break;
  }// end case 0
      
  // Record Mode (data)
  case 1: {
    Drone_sensor::ADC<3072> adc;        // create adc object with 3072 sample buffer
    adc.enableChannel(&chla);           // enable all channels
    adc.enableChannel(&pe);
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

    while (sensor.getMode() == 1) {
      while (!sensor.dataReady()) {}      // wait for data buffer to fill
      sensor.laser(sampleCounter % 2); // turn laser off every 256th sample (on otherwise)

      #ifdef DEBUG
        Serial.println(F("Getting data"));
      #endif // DEBUG
      chlaMsg.data = adc.getData(&chla);
      peMsg.data = adc.getData(&pe);
      ramanMsg.data = adc.getData(&raman);

      if(adc.OOR_flag){
        setMsg.data = (100*chla.config.gain) + (10*pe.config.gain) + (raman.config.gain);
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
      sensor.writeFile(chla);         // write data to SD card
      sensor.writeFile(pe);
      sensor.writeFile(raman);
      
      #ifndef DEBUG
        chlaPub.publish(&chlaMsg);    // publish chla data on the "sensor/chla" ROS topic
        pePub.publish(&peMsg);        // publish pe data
        ramanPub.publish(&ramanMsg);    // publish raman data
        nh.spinOnce();                 // update ROS
      #endif // !DEBUG

      #ifdef DEBUG
        Serial.print("Data:\t");
        Serial.print(chlaMsg.data);
        Serial.print("\t");
        Serial.print(peMsg.data);
        Serial.print("\t");
        Serial.println(ramanMsg.data);
        if (Serial.available()) {
            sensor.setMode(Serial.parseInt());
        }
      #endif // DEBUG
    }
    sensor.shutdown();
    break;
  } // end case 1
      
  // Calibrate (cali)
  case 2: {
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

    Drone_sensor::ADC<3072> adc;
    adc.enableChannel(&chla);
    adc.enableChannel(&pe);
    adc.enableChannel(&raman);
    sensor.initADC(&adc);

    float chla_bkgd = 0;
    float pe_bkgd = 0;
    float raman_bkgd = 0;
    float chla_blank[100] = {0};
    float pe_blank[100] = {0};
    float raman_blank[100] = {0};
    
    float chla_RMSE = 0;
    float pe_RMSE = 0;
    float raman_RMSE = 0;

    for (int i = 0; i < 100; i++) {
      sensor.start();
      while (!sensor.dataReady()) {}
      if(adc.OOR_flag){
        setMsg.data = (100*chla.config.gain) + (10*pe.config.gain) + (raman.config.gain);
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
      chla_blank[i] = adc.getData(&chla);
      pe_blank[i] = adc.getData(&pe);
      raman_blank[i] = adc.getData(&raman);

      sensor.start();
      while (!sensor.dataReady()) {}
      if(adc.OOR_flag){
        setMsg.data = (100*chla.config.gain) + (10*pe.config.gain) + (raman.config.gain);
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
      chla_blank[i] -= adc.getData(&chla);
      pe_blank[i] -= adc.getData(&pe);
      raman_blank[i] -= adc.getData(&raman);
      chla_bkgd += chla_blank[i];
      pe_bkgd += pe_blank[i];
      raman_bkgd += raman_blank[i];
      #ifndef DEBUG
        if (i % 10 == 0) {
            nh.spinOnce();
        }
      #endif // !DEBUG
    }

    chla_bkgd = chla_bkgd / 100.0;
    pe_bkgd = pe_bkgd / 100.0;
    raman_bkgd = raman_bkgd / 100.0;
    
    float chla_SSE = 0;
    float pe_SSE = 0;
    float raman_SSE = 0;

    for (int i = 0; i < 100; i++) {
        chla_SSE += sq(chla_blank[i] - chla_bkgd);
        pe_SSE += sq(pe_blank[i] - pe_bkgd);
        raman_SSE += sq(raman_blank[i] - raman_bkgd);
    }
    chla_RMSE = sqrt(chla_SSE / 100.0);
    pe_RMSE = sqrt(pe_SSE / 100.0);
    raman_RMSE = sqrt(raman_SSE / 100.0);

    sensor.file.println(F("Startup and Calibration"));
    sensor.file.print(F("RTC timestamp: "));
    sensor.file.println(startupTimeRTC.timestamp(DateTime::TIMESTAMP_FULL));
    sensor.file.print(F("ROS timestamp: "));
    sensor.file.println(startupTimeROS.timestamp(DateTime::TIMESTAMP_FULL));
    sensor.file.print(F("Chl a background (uV): "));
    sensor.file.println(chla_bkgd);
    sensor.file.print(F("Chl a RMSE noise (uV): "));
    sensor.file.println(chla_RMSE);
    sensor.file.print(F("PE background (uV): "));
    sensor.file.println(pe_bkgd);
    sensor.file.print(F("PE RMSE noise (uV): "));
    sensor.file.println(pe_RMSE);    
    sensor.file.print(F("Raman background (uV): "));
    sensor.file.println(raman_bkgd);
    sensor.file.print(F("Raman RMSE noise (uV): "));
    sensor.file.println(raman_RMSE);

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

  // if mode value is not recognized, set to 0
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