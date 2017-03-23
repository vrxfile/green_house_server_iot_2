#define BLYNK_PRINT Serial
#include <DHT.h>
#include <L3G.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <VirtualWire.h>
#include <UIPEthernet.h>
#include <BlynkSimpleUIPEthernet.h>
#include <BH1750FVI.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>

// Minimum soil moisture constant
#define MIN_SOIL_MOISTURE 30.0

// LED pin
#define LED_PIN      13

// Relay outputs
#define RELAY_PIN1   22
#define RELAY_PIN2   23
#define RELAY_PIN3   24
#define RELAY_PIN4   25
#define RELAY_PIN5   38

// Motor outputs
#define PWMA 4
#define DIRA 5
#define PWMB 6
#define DIRB 7

// Servo motor output
#define SERVO_PIN1 39
Servo servo_1;

// 433 MHz receiver parameters and vars
#define RECEIVER_PIN 3
#define TRANSMITTER_PIN 30
#define PTT_PIN 31

// DHT11 and DHT22 sensors
#define DHT11_PIN1 8
#define DHT11_PIN2 9
#define DHT22_PIN1 10
DHT dht11_1(DHT11_PIN1, DHT11);
DHT dht11_2(DHT11_PIN2, DHT11);
DHT dht22_1(DHT22_PIN1, DHT22);

// MQ-2 gas sensor pin
#define MQ2_PIN      A0

// Buttons pins
#define BUTTON_PIN1  A3
#define BUTTON_PIN2  A4
#define BUTTON_PIN3  A5

// Water level sensor
#define WATER_SIG_PIN A8
#define WATER_VCC_PIN A9

// HC-SR04 distance sensor pins
#define US1_trigPin 26
#define US1_echoPin 27
#define minimumRange 0
#define maximumRange 400

// Hardware reset pin
#define HRESETPIN    49

// LCD display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Period of software timers
#define LCD_UPDATE_TIME 10000         //old 10s
#define TERM_UPDATE_TIME 10000        //old 30s
#define LIGHT_UPDATE_TIME 10000       //old 60s
#define DHT_UPDATE_TIME 10000         //old 60s
#define REC_UPDATE_TIME 10            //old 10ms
#define RADIORESET_UPDATE_TIME 600000 //old 10min
#define IOT_TS_UPDATE_TIME 300000     //old 5min
#define IOT_TW_UPDATE_TIME 300000     //old 10s
#define BLYNK_UPDATE_UPDATE_TIME 100  //old 100ms
#define BLYNK_SEND_UPDATE_TIME 10000  //old 10s
#define PRESS_UPDATE_TIME 10000       //old 60s  
#define MAGNETIC_UPDATE_TIME 10000    //old 60s
#define SEISMO_UPDATE_TIME 10000      //old 60s
#define AUTO_UPDATE_TIME 1000         //old 1s
#define MQ2_UPDATE_TIME 10000         //old 60s
#define WATER_UPDATE_TIME 10000       //old 60s
#define HCSR04_UPDATE_TIME 5000       //old 3s
#define BUTTONS_UPDATE_TIME 1000      //old 1s
#define HRST_UPDATE_TIME 7200000      //old 2h

// Software timer counters
long timer_dht = 0;
long timer_light = 0;
long timer_press = 0;
long timer_magnetic = 0;
long timer_seismo = 0;
long timer_lcd = 0;
long timer_term = 0;
long timer_ts_iot = 0;
long timer_tw_iot = 0;
long timer_blynk_update = 0;
long timer_blynk_send = 0;
long timer_rec = 0;
long timer_mq2 = 0;
long timer_water = 0;
long timer_hcsr04 = 0;
long timer_buttons = 0;
long timer_radioreset = 0;
long timer_autocontrol = 0;
long timer_hreset = 0;

// Software watchdog 30 seconds
#define MAX_WDT 3000
long timer3_counter = 0;
long wdt_timer = 0;

// API key for Blynk
char auth[] = "d85ad70c9e7e41d9b29e55b080000070";
IPAddress blynk_ip(139, 59, 206, 133);

// Thingworx IoT server network parameters
char thingworx_server[] = "cttit5402.cloud.thingworx.com";
IPAddress thingworx_address(52, 87, 101, 142);
char thingworx_appKey[] = "3c0d1906-f333-4cfd-8c60-89408c06d121";
char thingworx_thingName[] = "MGBot_Greenhouse_Thing";
char thingworx_serviceName[] = "MGBotSetAllParameters";
const int thingworx_port = 80;

// IoT server sensor parameters
#define sensorCount 45
char* sensorNames[] = {"soil_temp1", "soil_temp2", "soil_temp3", "soil_temp4", "soil_temp5", "soil_temp6", "soil_temp7", "soil_temp8", "soil_temp9"
                       , "soil_moisture1", "soil_moisture2", "soil_moisture3", "soil_moisture4", "soil_moisture5", "soil_moisture6", "soil_moisture7", "soil_moisture8", "soil_moisture9"
                       , "air_temp1", "air_temp2", "air_temp3", "air_hum1", "air_hum2", "air_hum3", "air_pressure1", "sun_light1", "mag_x", "mag_y", "mag_z"
                       , "acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y", "gyr_z", "device_temp", "gas_conc", "motion_detect"
                       , "valve_timer1", "valve_timer2", "window_timer1", "lamps_timer1", "network_time", "radio_counter", "water_level"
                      };
float sensorValues[sensorCount] = {0};
#define SOIL_TEMP1     0
#define SOIL_TEMP2     1
#define SOIL_TEMP3     2
#define SOIL_TEMP4     3
#define SOIL_TEMP5     4
#define SOIL_TEMP6     5
#define SOIL_TEMP7     6
#define SOIL_TEMP8     7
#define SOIL_TEMP9     8
#define SOIL_MOISTURE1 9
#define SOIL_MOISTURE2 10
#define SOIL_MOISTURE3 11
#define SOIL_MOISTURE4 12
#define SOIL_MOISTURE5 13
#define SOIL_MOISTURE6 14
#define SOIL_MOISTURE7 15
#define SOIL_MOISTURE8 16
#define SOIL_MOISTURE9 17
#define AIR_TEMP1      18
#define AIR_TEMP2      19
#define AIR_TEMP3      20
#define AIR_HUM1       21
#define AIR_HUM2       22
#define AIR_HUM3       23
#define AIR_PRESSURE1  24
#define SUN_LIGHT1     25
#define MAG_X          26
#define MAG_Y          27
#define MAG_Z          28
#define ACC_X          29
#define ACC_Y          30
#define ACC_Z          31
#define GYR_X          32
#define GYR_Y          33
#define GYR_Z          34
#define DEVICE_TEMP    35
#define GAS_CONC       36
#define MOTION_DETECT  37
#define VALVE_TIMER1   38
#define VALVE_TIMER2   39
#define WINDOW_TIMER1  40
#define LAMPS_TIMER1   41
#define NETWORK_TIME   42
#define RADIO_COUNTER  43
#define WATER_LEVEL    44

// Read flags for radio sensors
uint8_t sensorFlags[sensorCount] = {0};

// States of control devices
#define controlCount 4
int controlValues[controlCount] = {0};
#define VALVE_POWER1  0
#define VALVE_POWER2  1
#define WINDOW_STATE1 2
#define LAMP_POWER1   3
// Flags of control devices
uint8_t controlFlags[controlCount] = {0};
// Timers for control devices
int controlTimers[controlCount] = {0};
// Counters for control devices
#define MAX_CONTROL_COUNT 50
int controlCounters[controlCount] = {0};

// Control states from ThingWorx
int valve_control1 = 0;
int valve_control2 = 0;
int window_control1 = 0;
int lamps_control1 = 0;
int old_valve_control1 = 0;
int old_valve_control2 = 0;
int old_window_control1 = 0;
int old_lamps_control1 = 0;

// Addresses of radio sensors
#define MIN_RADIO_ADDR 1
#define MAX_RADIO_ADDR 9

// States and flags of buttons
#define buttonCount 3
uint8_t buttonStates[buttonCount] = {0};
uint8_t buttonFlags[buttonCount] = {0};
#define BUTTON1  0
#define BUTTON2  1
#define BUTTON3  2

// Timeouts of IoT server
#define IOT_TIMEOUT1 5000
#define IOT_TIMEOUT2 100

// Receive timer from IoT server
long timer_iot_timeout = 0;

// Receive buffer length (from IoT server)
#define BUFF_LENGTH 256

// Receive buffer (from IoT server)
char buff[BUFF_LENGTH] = "";

// Parameters for network adapter
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xAE, 0xCD};
// Local IP if DHCP fails
IPAddress ip(192, 168, 1, 250);
IPAddress dnsServerIP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// Ethernet network object
EthernetClient client;

// Page number for LCD display
int page_number = 0;

// Object definition for light sensor
BH1750FVI LightSensor_1;

// For ThingSpeak IoT
const String CHANNELID_1 = "128676";
const String WRITEAPIKEY_1 = "I77EPRC4GSBK2VH9";
const String CHANNELID_2 = "128677";
const String WRITEAPIKEY_2 = "AGSEI25MA1010BNE";
const String CHANNELID_3 = "128678";
const String WRITEAPIKEY_3 = "YVWTNI5L0NXV5XER";
const String CHANNELID_4 = "128680";
const String WRITEAPIKEY_4 = "9GS2VWQ661JM2PHU";
const String CHANNELID_5 = "131612";
const String WRITEAPIKEY_5 = "RFWNIFRUD1SFTVK9";
IPAddress thingspeak_server(184, 106, 153, 149);
const int thingspeak_port = 80;

// Orientation sensor
L3G gyro;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(15883);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(10345);

void setup()
{
  // Init serial port
  Serial.begin(115200);
  while (!Serial) {}

  // Init onboard LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Init hardware RESET pin
  pinMode(HRESETPIN, INPUT);

  // Init MQ-2 gas sensor pin
  pinMode(MQ2_PIN, INPUT);

  // Init HC-SR04 distance sensor pins
  pinMode(US1_trigPin, OUTPUT);
  pinMode(US1_echoPin, INPUT);

  // Init buttons pins with pullup's
  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  pinMode(BUTTON_PIN3, INPUT_PULLUP);

  // Init water level sensor pins
  pinMode(WATER_SIG_PIN, INPUT);
  pinMode(WATER_VCC_PIN, OUTPUT);
  digitalWrite(WATER_VCC_PIN, false);

  // Init relay outputs
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);
  pinMode(RELAY_PIN3, OUTPUT);
  pinMode(RELAY_PIN4, OUTPUT);
  digitalWrite(RELAY_PIN5, HIGH);
  pinMode(RELAY_PIN5, OUTPUT);

  // Init motor outputs
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Init servo motor output
  servo_1.attach(SERVO_PIN1);

  // Timer 3 interrupt (for custom WatchDog)
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  // Set timer1_counter to the correct value for our interrupt interval
  timer3_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer3_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer3_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  TCNT3 = timer3_counter;   // preload timer
  TCCR3B |= (1 << CS12);    // 256 prescaler
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
  watchdog_reset();

  // Init receiver
  vw_set_rx_pin(RECEIVER_PIN);
  vw_set_tx_pin(TRANSMITTER_PIN);
  vw_set_ptt_pin(PTT_PIN);
  vw_setup(512); watchdog_reset();
  vw_rx_start(); watchdog_reset();

  // Init DHT11 and DHT22 sensors
  dht11_1.begin(); watchdog_reset();
  dht11_2.begin(); watchdog_reset();
  dht22_1.begin(); watchdog_reset();

  // Init BH1750 light sensor
  LightSensor_1.begin(); watchdog_reset();
  LightSensor_1.setMode(Continuously_High_Resolution_Mode); watchdog_reset();

  // Init BMP085 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor!");
  }
  watchdog_reset();

  // Init L3G4200D sensor
  if (!gyro.init()) {
    Serial.println("Failed to autodetect gyro type!");
  } else {
    gyro.enableDefault();
  }
  watchdog_reset();

  // Init HMC5883L sensor
  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor!");
  }
  watchdog_reset();

  // Init ADXL345 sensor
  if (!accel.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor!");
  } else {
    accel.setRange(ADXL345_RANGE_2_G);
    accel.setDataRate(ADXL345_DATARATE_200_HZ);
  }
  watchdog_reset();

  // Init LCD
  lcd.init(); watchdog_reset();
  lcd.backlight(); watchdog_reset();
  lcd.clear(); watchdog_reset();

  // Init Ethernet ENC28J60
  //  if (Ethernet.begin(mac) == 0)
  //  {
  //    Serial.println("Failed to configure Ethernet using DHCP");
  //    Ethernet.begin(mac, ip, dnsServerIP, gateway, subnet);
  //  }
  //  watchdog_reset();
  //  Serial.print("LocalIP:\t\t");
  //  Serial.println(Ethernet.localIP());
  //  Serial.print("SubnetMask:\t\t");
  //  Serial.println(Ethernet.subnetMask());
  //  Serial.print("GatewayIP:\t\t");
  //  Serial.println(Ethernet.gatewayIP());
  //  Serial.print("dnsServerIP:\t\t");
  //  Serial.println(Ethernet.dnsServerIP());
  //  Serial.println("");
  //  watchdog_reset();

  // Init Blynk on ENC28J60
  //Blynk.begin(auth);
  Blynk.begin(auth, blynk_ip, 8442);
  watchdog_reset();

  // One time read all sensors and print data
  readSensorDHT11_1(); watchdog_reset();
  readSensorDHT11_2(); watchdog_reset();
  readSensorDHT22_1(); watchdog_reset();
  readSensorBH1750_1(); watchdog_reset();
  readSensorBMP085_1(); watchdog_reset();
  readSensorHMC5883L_1(); watchdog_reset();
  readSensorACCGYRO_1(); watchdog_reset();
  readSensorMQ2(); watchdog_reset();
  readSensorHCSR04(); watchdog_reset();
  readSensorWater(); watchdog_reset();
  lcd.clear();
  lcd.setCursor(0, 0); lcd_printstr("OUT_T = " + String(sensorValues[AIR_TEMP3], 1) + " *C");
  lcd.setCursor(0, 1); lcd_printstr("OUT_H = " + String(sensorValues[AIR_HUM3], 1) + " %");
  lcd.setCursor(0, 2); lcd_printstr("PRESS = " + String(sensorValues[AIR_PRESSURE1], 1) + " mm");
  lcd.setCursor(0, 3); lcd_printstr("LIGHT = " + String(sensorValues[SUN_LIGHT1], 1) + " LX");
  watchdog_reset();
  printSerialData();
  watchdog_reset();
}

void loop()
{
  // Read DHT11 and DHT22 senors
  if (millis() > timer_dht + DHT_UPDATE_TIME)
  {
    // Read sensors
    readSensorDHT11_1(); watchdog_reset();
    readSensorDHT11_2(); watchdog_reset();
    readSensorDHT22_1(); watchdog_reset();
    // Print data to terminal
    Serial.println("DHT11_1: " + String(sensorValues[AIR_TEMP1], 3) + " *C\t\t" + String(sensorValues[AIR_HUM1], 3) + " %");
    Serial.println("DHT11_2: " + String(sensorValues[AIR_TEMP2], 3) + " *C\t\t" + String(sensorValues[AIR_HUM2], 3) + " %");
    Serial.println("DHT22_1: " + String(sensorValues[AIR_TEMP3], 3) + " *C\t\t" + String(sensorValues[AIR_HUM3], 3) + " %");
    // Reset timer
    timer_dht = millis();
  }

  // Read BH1750 light sensor
  if (millis() > timer_light + LIGHT_UPDATE_TIME)
  {
    // Read sensor
    readSensorBH1750_1(); watchdog_reset();
    // Print data to terminal
    Serial.println("BH1750: " + String(sensorValues[SUN_LIGHT1], 3) + " lux");
    // Reset timer
    timer_light = millis();
  }

  // Read BMP085 pressure sensor
  if (millis() > timer_press + PRESS_UPDATE_TIME)
  {
    // Read sensor
    readSensorBMP085_1(); watchdog_reset();
    // Print data to terminal
    Serial.println("BMP085: " + String(sensorValues[AIR_PRESSURE1], 3) + " mm Hg\t\t" + String(sensorValues[DEVICE_TEMP], 3) + " *C");
    // Reset timer
    timer_press = millis();
  }

  // Read HMC5883L magnetic sensor
  if (millis() > timer_magnetic + MAGNETIC_UPDATE_TIME)
  {
    // Read sensor
    readSensorHMC5883L_1(); watchdog_reset();
    // Print data to terminal
    Serial.println("HMC5883L: " + String(sensorValues[MAG_X], 3) + " uT\t\t" + String(sensorValues[MAG_Y], 3) + " uT\t\t" + String(sensorValues[MAG_Z], 3) + " uT");
    // Reset timer
    timer_magnetic = millis();
  }

  // Read accelerometer and gyroscope sensors
  if (millis() > timer_seismo + SEISMO_UPDATE_TIME)
  {
    // Read sensor
    readSensorACCGYRO_1(); watchdog_reset();
    // Print data to terminal
    Serial.println("ADXL345: " + String(sensorValues[ACC_X], 3) + " m/s^2\t\t" + String(sensorValues[ACC_Y], 3) + " m/s^2\t\t" + String(sensorValues[ACC_Z], 3) + " m/s^2");
    Serial.println("L3G4200D: " + String(sensorValues[GYR_X], 3) + " grad/s\t\t" + String(sensorValues[GYR_Y], 3) + " grad/s\t\t" + String(sensorValues[GYR_Z], 3) + " grad/s");
    // Reset timer
    timer_seismo = millis();
  }

  // Read MQ-2 gas sensor
  if (millis() > timer_mq2 + MQ2_UPDATE_TIME)
  {
    // Read sensor
    readSensorMQ2(); watchdog_reset();
    // Print data to terminal
    Serial.println("MQ-2: " + String(sensorValues[GAS_CONC], 3) + " %");
    // Reset timer
    timer_mq2 = millis();
  }

  // Read water level sensor
  if (millis() > timer_water + WATER_UPDATE_TIME)
  {
    // Read sensor
    readSensorWater(); watchdog_reset();
    // Print data to terminal
    Serial.println("Water level: " + String(sensorValues[WATER_LEVEL], 3));
    // Reset timer
    timer_water = millis();
  }

  // Read HC-SR04 distance sensor
  if (millis() > timer_hcsr04 + HCSR04_UPDATE_TIME)
  {
    // Read sensor
    readSensorHCSR04(); watchdog_reset();
    // Detect object and switch on lamps
    if ((sensorValues[MOTION_DETECT] < 50))
    {
      // Increment lamps timer
      controlTimers[LAMP_POWER1] = 60;
      sensorValues[LAMPS_TIMER1] = controlTimers[LAMP_POWER1];
      sensorFlags[LAMPS_TIMER1] = true;
      watchdog_reset();
    }
    // Print data to terminal
    Serial.println("HC-SR04: " + String(sensorValues[MOTION_DETECT], 3) + " cm");
    // Reset timer
    timer_hcsr04 = millis();
  }

  // Read buttons
  if (millis() > timer_buttons + BUTTONS_UPDATE_TIME)
  {
    // Read buttons
    readButtons(); watchdog_reset();
    // Increment timer of input valve
    if ((buttonStates[BUTTON1] == 0))
    {
      controlTimers[VALVE_POWER1] = controlTimers[VALVE_POWER1] + 50;
      sensorValues[VALVE_TIMER1] = controlTimers[VALVE_POWER1];
      sensorFlags[VALVE_TIMER1] = true;
    }
    // Increment timer of output valve
    if ((buttonStates[BUTTON2] == 0))
    {
      controlTimers[VALVE_POWER2] = controlTimers[VALVE_POWER2] + 50;
      sensorValues[VALVE_TIMER2] = controlTimers[VALVE_POWER2];
      sensorFlags[VALVE_TIMER2] = true;
    }
    // Switch off all valves
    if ((buttonStates[BUTTON3] == 0))
    {
      controlTimers[VALVE_POWER1] = 0;
      controlTimers[VALVE_POWER2] = 0;
      sensorValues[VALVE_TIMER1] = controlTimers[VALVE_POWER1];
      sensorValues[VALVE_TIMER2] = controlTimers[VALVE_POWER2];
      sensorFlags[VALVE_TIMER1] = false;
      sensorFlags[VALVE_TIMER2] = false;
    }
    watchdog_reset();
    // Print control device timers
    if ((buttonStates[BUTTON1] == 0) || (buttonStates[BUTTON2] == 0) || (buttonStates[BUTTON3] == 0))
    {
      // Print control timers to LCD
      lcd.clear();
      lcd.setCursor(0, 0); lcd_printstr("VALVE1 = " + String(controlTimers[VALVE_POWER1]));
      lcd.setCursor(0, 1); lcd_printstr("VALVE2 = " + String(controlTimers[VALVE_POWER2]));
      lcd.setCursor(0, 2); lcd_printstr("WINDOW = " + String(controlTimers[WINDOW_STATE1]));
      lcd.setCursor(0, 3); lcd_printstr("LAMPS  = " + String(controlTimers[LAMP_POWER1]));
      watchdog_reset();
    }
    // Print data to terminal
    Serial.println("Buttons: " + String(buttonStates[BUTTON1]) + " " + String(buttonStates[BUTTON2]) + " " + String(buttonStates[BUTTON3]));
    // Reset timer
    timer_buttons = millis();
  }

  // Reset flags of radio sensors
  if (millis() > timer_radioreset + RADIORESET_UPDATE_TIME)
  {
    // Reset flags
    for (int u = 0; u < sensorCount; u++)
    {
      sensorFlags[u] = false;
    }
    watchdog_reset();
    // Reset timer
    timer_radioreset = millis();
  }

  // Automatic function control
  if (millis() > timer_autocontrol + AUTO_UPDATE_TIME)
  {
    // Run automatic function control
    controlDevices_1(); watchdog_reset();
    // Reset timer
    timer_autocontrol = millis();
  }

  // Read 433 MHz receiver
  if (millis() > timer_rec + REC_UPDATE_TIME)
  {
    uint8_t rec_buf[VW_MAX_MESSAGE_LEN];
    uint8_t rec_buflen = VW_MAX_MESSAGE_LEN;
    if (vw_get_message(rec_buf, &rec_buflen))
    {
      digitalWrite(LED_PIN, HIGH);
      Serial.print("Received: ");
      for (int i = 0; i < rec_buflen; i++)
      {
        Serial.print((char)rec_buf[i]);
      }
      Serial.println();
      watchdog_reset();
      // Decrypt parameters from JSON packet
      for (int i = 0; i < rec_buflen; i++)
      {
        buff[i] = (char)rec_buf[i];
      }
      buff[rec_buflen] = 0x00;
      char sensor_type = buff[8];
      StaticJsonBuffer<BUFF_LENGTH> jsonBuffer;
      JsonObject& json_array = jsonBuffer.parseObject(buff);
      int sensor_addr = json_array["A"];
      float sensor_moisture = json_array["M"];
      float sensor_temperature = json_array["T"];
      watchdog_reset();
      if ((sensor_addr >= MIN_RADIO_ADDR) && (sensor_addr <= MAX_RADIO_ADDR))
      {
        Serial.println("Node address: " + String(sensor_addr));
        Serial.println("Sensor type: " + String(sensor_type));
        Serial.println("Moisture data: " + String(sensor_moisture, 1));
        Serial.println("Temperature data: " + String(sensor_temperature, 1));
        Serial.println();
        if (sensor_type == 'T')
        {
          sensorValues[sensor_addr - 1] = sensor_temperature;
          sensorFlags[sensor_addr - 1] = true;
        }
        else if (sensor_type == 'M')
        {
          sensorValues[sensor_addr + 8] = sensor_moisture;
          sensorFlags[sensor_addr + 8] = true;
        }
        watchdog_reset();
        // Automatic on output valve if minimum moisture
        float min_moisture = 100;
        int flag_moisture = false;
        for (int u = SOIL_MOISTURE1; u <= SOIL_MOISTURE9; u++)
        {
          if (sensorFlags[u])
          {
            flag_moisture = true;
            if (sensorValues[u] < min_moisture)
            {
              min_moisture = sensorValues[u];
            }
          }
        }
        watchdog_reset();
        if ((min_moisture < MIN_SOIL_MOISTURE) && (flag_moisture))
        {
          controlCounters[VALVE_POWER2] = controlCounters[VALVE_POWER2] + 1;
          if (controlCounters[VALVE_POWER2] <= MAX_CONTROL_COUNT)
          {
            controlTimers[VALVE_POWER2] = 300;
          }
        }
        if ((min_moisture >= MIN_SOIL_MOISTURE) && (flag_moisture))
        {
          controlCounters[VALVE_POWER2] = 0;
        }
        Serial.println("Min soil moisture: " + String(min_moisture, 1));
        Serial.println();
        watchdog_reset();
        // Increment radio packets couner
        sensorValues[RADIO_COUNTER] = sensorValues[RADIO_COUNTER] + 1;
        sensorFlags[RADIO_COUNTER] = true;
      }
      digitalWrite(LED_PIN, LOW);
      watchdog_reset();
    }
    watchdog_reset();
    // Reset timer
    timer_rec = millis();
  }

  // Print data to LCD
  if (millis() > timer_lcd + LCD_UPDATE_TIME)
  {
    switch (page_number)
    {
      case 0:
        lcd.clear();
        lcd.setCursor(0, 0); lcd_printstr("IN_T1 = " + String(sensorValues[AIR_TEMP1], 1) + " *C");
        lcd.setCursor(0, 1); lcd_printstr("IN_H1 = " + String(sensorValues[AIR_HUM1], 1) + " %");
        lcd.setCursor(0, 2); lcd_printstr("IN_T2 = " + String(sensorValues[AIR_TEMP2], 1) + " *C");
        lcd.setCursor(0, 3); lcd_printstr("IN_H2 = " + String(sensorValues[AIR_HUM2], 1) + " %");
        watchdog_reset();
        break;
      case 1:
        lcd.clear();
        lcd.setCursor(0, 0); lcd_printstr("OUT_T = " + String(sensorValues[AIR_TEMP3], 1) + " *C");
        lcd.setCursor(0, 1); lcd_printstr("OUT_H = " + String(sensorValues[AIR_HUM3], 1) + " %");
        lcd.setCursor(0, 2); lcd_printstr("PRESS = " + String(sensorValues[AIR_PRESSURE1], 1) + " mm");
        lcd.setCursor(0, 3); lcd_printstr("LIGHT = " + String(sensorValues[SUN_LIGHT1], 1) + " LX");
        watchdog_reset();
        break;
      case 2:
        lcd.clear();
        if (sensorFlags[SOIL_TEMP1]) {
          lcd.setCursor(0, 0); lcd_printstr("R_T1 = " + String(sensorValues[SOIL_TEMP1], 1) + " *C");
        } else {
          lcd.setCursor(0, 0); lcd_printstr("R_T1 = NO DATA");
        }
        if (sensorFlags[SOIL_MOISTURE1]) {
          lcd.setCursor(0, 1); lcd_printstr("R_M1 = " + String(sensorValues[SOIL_MOISTURE1], 1) + " %");
        } else {
          lcd.setCursor(0, 1); lcd_printstr("R_M1 = NO DATA");
        }
        if (sensorFlags[SOIL_TEMP2]) {
          lcd.setCursor(0, 2); lcd_printstr("R_T2 = " + String(sensorValues[SOIL_TEMP2], 1) + " *C");
        } else {
          lcd.setCursor(0, 2); lcd_printstr("R_T2 = NO DATA");
        }
        if (sensorFlags[SOIL_MOISTURE2]) {
          lcd.setCursor(0, 3); lcd_printstr("R_M2 = " + String(sensorValues[SOIL_MOISTURE2], 1) + " %");
        } else {
          lcd.setCursor(0, 3); lcd_printstr("R_M2 = NO DATA");
        }
        watchdog_reset();
        break;
      case 3:
        lcd.clear();
        if (sensorFlags[SOIL_TEMP3]) {
          lcd.setCursor(0, 0); lcd_printstr("R_T3 = " + String(sensorValues[SOIL_TEMP3], 1) + " *C");
        } else {
          lcd.setCursor(0, 0); lcd_printstr("R_T3 = NO DATA");
        }
        if (sensorFlags[SOIL_MOISTURE3]) {
          lcd.setCursor(0, 1); lcd_printstr("R_M3 = " + String(sensorValues[SOIL_MOISTURE3], 1) + " %");
        } else {
          lcd.setCursor(0, 1); lcd_printstr("R_M3 = NO DATA");
        }
        if (sensorFlags[SOIL_TEMP4]) {
          lcd.setCursor(0, 2); lcd_printstr("R_T4 = " + String(sensorValues[SOIL_TEMP4], 1) + " *C");
        } else {
          lcd.setCursor(0, 2); lcd_printstr("R_T4 = NO DATA");
        }
        if (sensorFlags[SOIL_MOISTURE4]) {
          lcd.setCursor(0, 3); lcd_printstr("R_M4 = " + String(sensorValues[SOIL_MOISTURE4], 1) + " %");
        } else {
          lcd.setCursor(0, 3); lcd_printstr("R_M4 = NO DATA");
        }
        watchdog_reset();
        break;
      case 4:
        lcd.clear();
        lcd.setCursor(0, 0); lcd_printstr("MX = " + String(sensorValues[MAG_X], 1) + " uT");
        lcd.setCursor(0, 1); lcd_printstr("MY = " + String(sensorValues[MAG_Y], 1) + " uT");
        lcd.setCursor(0, 2); lcd_printstr("MZ = " + String(sensorValues[MAG_Z], 1) + " uT");
        lcd.setCursor(0, 3); lcd_printstr("DT = " + String(sensorValues[DEVICE_TEMP], 1) + " *C");
        watchdog_reset();
        break;
      case 5:
        lcd.clear();
        lcd.setCursor(0, 0); lcd_printstr("VALVE1 = " + String(controlTimers[VALVE_POWER1]));
        lcd.setCursor(0, 1); lcd_printstr("VALVE2 = " + String(controlTimers[VALVE_POWER2]));
        lcd.setCursor(0, 2); lcd_printstr("WINDOW = " + String(controlTimers[WINDOW_STATE1]));
        lcd.setCursor(0, 3); lcd_printstr("LAMPS  = " + String(controlTimers[LAMP_POWER1]));
        watchdog_reset();
        break;
      case 6:
        lcd.clear();
        lcd.setCursor(0, 0); lcd_printstr("GAS  = " + String(sensorValues[GAS_CONC], 1) + " %");
        lcd.setCursor(0, 1); lcd_printstr("DIST = " + String(sensorValues[MOTION_DETECT], 1) + " cm");
        lcd.setCursor(0, 2); lcd_printstr("NETT = " + String(sensorValues[NETWORK_TIME], 0) + " ms");
        lcd.setCursor(0, 3); lcd_printstr("RCNT = " + String(sensorValues[RADIO_COUNTER], 0));
        watchdog_reset();
        break;
      case 7:
        lcd.clear();
        lcd.setCursor(0, 0); lcd_printstr("WATER = " + String(controlTimers[WATER_LEVEL]));
        watchdog_reset();
        break;
      default:
        break;
    }
    // Change page number
    page_number++;
    if (page_number > 7)
    {
      page_number = 0;
    }
    watchdog_reset();
    // Reset timer
    timer_lcd = millis();
  }

  // Print data to terminal
  if (millis() > timer_term + TERM_UPDATE_TIME)
  {
    // Print all data to serial terminal
    printSerialData(); watchdog_reset();
    // Reset timer
    timer_term = millis();
  }

  // Send data to ThingWorx IoT server
  if (millis() > timer_tw_iot + IOT_TW_UPDATE_TIME)
  {
    // Print message to LCD
    lcd.clear();
    lcd.setCursor(0, 0); lcd_printstr("Sending data 1...");
    lcd.setCursor(0, 1); lcd_printstr("Please wait...");
    lcd.setCursor(0, 2); lcd_printstr("0%");
    watchdog_reset();
    // Send data to ThingWorx and receive control packets
    sendDataIot_ThingWorx_1();
    watchdog_reset();
    lcd.setCursor(0, 2); lcd_printstr("100%");
    lcd.setCursor(0, 3); lcd_printstr("Data sent OK!");
    watchdog_reset();
    // Reset timer
    timer_tw_iot = millis();
  }

  // Send data to ThingSpeak IoT server and measure send data time
  if (millis() > timer_ts_iot + IOT_TS_UPDATE_TIME)
  {
    unsigned long beg_timer = 0;
    unsigned long end_timer = 0;
    float network_time = 0;
    // Print message to LCD
    lcd.clear();
    lcd.setCursor(0, 0); lcd_printstr("Sending data 2...");
    lcd.setCursor(0, 1); lcd_printstr("Please wait...");
    lcd.setCursor(0, 2); lcd_printstr("0%");
    watchdog_reset();
    // Send air sensors data
    beg_timer = millis();
    sendDataIot_ThingSpeak_1();
    end_timer = millis() - beg_timer;
    network_time = network_time + end_timer;
    watchdog_reset();
    lcd.setCursor(0, 2); lcd_printstr("20%"); watchdog_reset();
    // Send soil temperature sensors data
    beg_timer = millis();
    sendDataIot_ThingSpeak_2();
    end_timer = millis() - beg_timer;
    network_time = network_time + end_timer;
    watchdog_reset();
    lcd.setCursor(0, 2); lcd_printstr("40%"); watchdog_reset();
    // Send soil moisture sensors data
    beg_timer = millis();
    sendDataIot_ThingSpeak_3();
    end_timer = millis() - beg_timer;
    network_time = network_time + end_timer;
    watchdog_reset();
    lcd.setCursor(0, 2); lcd_printstr("60%"); watchdog_reset();
    // Send controls data
    beg_timer = millis();
    sendDataIot_ThingSpeak_4();
    end_timer = millis() - beg_timer;
    network_time = network_time + end_timer;
    watchdog_reset();
    lcd.setCursor(0, 2); lcd_printstr("80%"); watchdog_reset();
    // Send magnetic and seismo data
    beg_timer = millis();
    sendDataIot_ThingSpeak_5();
    end_timer = millis() - beg_timer;
    network_time = network_time + end_timer;
    watchdog_reset();
    lcd.setCursor(0, 2); lcd_printstr("100%");
    lcd.setCursor(0, 3); lcd_printstr("Data sent OK!");
    watchdog_reset();
    // Clear control flags
    for (int u = 0; u < controlCount; u++)
    {
      controlFlags[u] = false;
    }
    // Clear radio packets counter
    sensorValues[RADIO_COUNTER] = 0;
    sensorFlags[RADIO_COUNTER] = false;
    // Network data send time
    sensorValues[NETWORK_TIME] = network_time;
    sensorFlags[NETWORK_TIME] = true;
    // Reset timer
    timer_ts_iot = millis();
  }

  // Blynk update
  if (millis() > timer_blynk_update + BLYNK_UPDATE_UPDATE_TIME)
  {
    // Update Blynk
    Blynk.run();
    // Reset timer
    timer_blynk_update = millis();
  }

  // Send data to blynk
  if (millis() > timer_blynk_send + BLYNK_SEND_UPDATE_TIME)
  {
    // Send data to blynk
    Serial.print("Sending data to Blynk...");
    Blynk.virtualWrite(V0, sensorValues[AIR_TEMP1]); delay(50); Serial.print(" 10%");
    Blynk.virtualWrite(V1, sensorValues[AIR_TEMP2]); delay(50); Serial.print(" 20%");
    Blynk.virtualWrite(V2, sensorValues[AIR_TEMP3]); delay(50); Serial.print(" 30%");
    Blynk.virtualWrite(V3, sensorValues[AIR_HUM1]); delay(50); Serial.print(" 40%");
    Blynk.virtualWrite(V4, sensorValues[AIR_HUM2]); delay(50); Serial.print(" 50%");
    Blynk.virtualWrite(V5, sensorValues[AIR_HUM3]); delay(50); Serial.print(" 60%");
    Blynk.virtualWrite(V6, sensorValues[AIR_PRESSURE1]); delay(50); Serial.print(" 70%");
    Blynk.virtualWrite(V7, sensorValues[SUN_LIGHT1]); delay(50); Serial.print(" 80%");
    Serial.println(" 100%");
    Serial.println("Data successfully sent!");
    Serial.println();
    // Reset timer
    timer_blynk_send = millis();
  }

  // Hard reset of device
  if (millis() > timer_hreset + HRST_UPDATE_TIME)
  {
    Serial.println("Hard reset timeout!\n");
    while (1)
    {
    }
  }

  // Reset software watchdog
  watchdog_reset();
}

// Read DHT11 sensor #1
void readSensorDHT11_1()
{
  sensorValues[AIR_HUM1] = dht11_1.readHumidity();
  sensorValues[AIR_TEMP1] = dht11_1.readTemperature();
  sensorFlags[AIR_HUM1] = true;
  sensorFlags[AIR_TEMP1] = true;
}

// Read DHT11 sensor #2
void readSensorDHT11_2()
{
  sensorValues[AIR_HUM2] = dht11_2.readHumidity();
  sensorValues[AIR_TEMP2] = dht11_2.readTemperature();
  sensorFlags[AIR_HUM2] = true;
  sensorFlags[AIR_TEMP2] = true;
}

// Read DHT22 sensor #1
void readSensorDHT22_1()
{
  sensorValues[AIR_HUM3] = dht22_1.readHumidity();
  sensorValues[AIR_TEMP3] = dht22_1.readTemperature();
  sensorFlags[AIR_HUM3] = true;
  sensorFlags[AIR_TEMP3] = true;
}

// Read BH1750 light sensor #1
void readSensorBH1750_1()
{
  sensorValues[SUN_LIGHT1] = LightSensor_1.getAmbientLight();
  sensorFlags[SUN_LIGHT1] = true;
}

// Read BMP085 air pressure sensor #1
void readSensorBMP085_1()
{
  float ttt = 0;
  sensors_event_t p_event;
  bmp.getEvent(&p_event);
  if (p_event.pressure)
  {
    sensorValues[AIR_PRESSURE1] = p_event.pressure * 7.5006 / 10;
    bmp.getTemperature(&ttt);
    sensorValues[DEVICE_TEMP] = ttt;
    sensorFlags[AIR_PRESSURE1] = true;
    sensorFlags[DEVICE_TEMP] = true;
  }
  else
  {
    sensorFlags[AIR_PRESSURE1] = false;
    sensorFlags[DEVICE_TEMP] = false;
  }
}

// Read HMC5883L magnetic sensor #1
void readSensorHMC5883L_1()
{
  sensors_event_t m_event;
  mag.getEvent(&m_event);
  sensorValues[MAG_X] = m_event.magnetic.x;
  sensorValues[MAG_Y] = m_event.magnetic.y;
  sensorValues[MAG_Z] = m_event.magnetic.z;
  sensorFlags[MAG_X] = true;
  sensorFlags[MAG_Y] = true;
  sensorFlags[MAG_Z] = true;
}

// Read accelerometer and gyroscope sensors
void readSensorACCGYRO_1()
{
  sensors_event_t a_event;
  accel.getEvent(&a_event);
  sensorValues[ACC_X] = a_event.acceleration.x;
  sensorValues[ACC_Y] = a_event.acceleration.y;
  sensorValues[ACC_Z] = a_event.acceleration.z;
  gyro.read();
  sensorValues[GYR_X] = gyro.g.x / 1000;
  sensorValues[GYR_Y] = gyro.g.y / 1000;
  sensorValues[GYR_Z] = gyro.g.z / 1000;
  sensorFlags[ACC_X] = true;
  sensorFlags[ACC_Y] = true;
  sensorFlags[ACC_Z] = true;
  sensorFlags[GYR_X] = true;
  sensorFlags[GYR_Y] = true;
  sensorFlags[GYR_Z] = true;
}

// Read MQ-2 gas sensor
void readSensorMQ2()
{
  sensorValues[GAS_CONC] = analogRead(MQ2_PIN) / 1023.0 * 100.0;
  sensorFlags[GAS_CONC] = true;
}

// Read water level sensor
void readSensorWater()
{
  digitalWrite(WATER_VCC_PIN, true);
  sensorValues[WATER_LEVEL] = analogRead(WATER_SIG_PIN) / 1023.0 * 100.0;
  digitalWrite(WATER_VCC_PIN, false);
  sensorFlags[WATER_LEVEL] = true;
}

// Read HC-SR04 distance sensor
void readSensorHCSR04()
{
  float duration = 0;
  float distance = 0;
  digitalWrite(US1_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(US1_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(US1_trigPin, LOW);
  duration = pulseIn(US1_echoPin, HIGH, 50000);
  distance = duration / 58.2;
  if (distance >= maximumRange || distance <= minimumRange)
  {
    distance = -1;
  }
  sensorValues[MOTION_DETECT] = distance;
  if (distance < 1)
  {
    sensorFlags[MOTION_DETECT] = false;
  }
  else
  {
    sensorFlags[MOTION_DETECT] = true;
  }
}

// Read buttons
void readButtons()
{
  buttonStates[BUTTON1] = digitalRead(BUTTON_PIN1);
  buttonStates[BUTTON2] = digitalRead(BUTTON_PIN2);
  buttonStates[BUTTON3] = digitalRead(BUTTON_PIN3);
  buttonFlags[BUTTON1] = buttonStates[BUTTON1];
  buttonFlags[BUTTON2] = buttonStates[BUTTON2];
  buttonFlags[BUTTON3] = buttonStates[BUTTON3];
}

// Print all sensors data to serial terminal
void printSerialData()
{
  for (int u = 0; u < sensorCount; u++)
  {
    Serial.print("Sensor");
    Serial.print(u);
    Serial.print(": ");
    Serial.print(String(sensorValues[u], 1) + "\t\t");
    Serial.println(String(sensorFlags[u]));
  }
  Serial.println();
  for (int u = 0; u < controlCount; u++)
  {
    Serial.print("Control");
    Serial.print(u);
    Serial.print(": ");
    Serial.print(String(controlTimers[u]) + "\t\t");
    Serial.print(String(controlValues[u]) + "\t\t");
    Serial.print(String(controlFlags[u]) + "\t\t");
    Serial.println(String(controlCounters[u]));
  }
  Serial.println();
}

// LCD print string function
void lcd_printstr(String str1)
{
  for (int i = 0; i < str1.length(); i++)
  {
    lcd.print(str1.charAt(i));
  }
}

// Send air sensors data to ThingSpeak
void sendDataIot_ThingSpeak_1()
{
  Serial.print("Connecting to ");
  Serial.print(thingspeak_server);
  Serial.println("...");
  if (client.connect(thingspeak_server, thingspeak_port))
  {
    watchdog_reset();
    if (client.connected())
    {
      watchdog_reset();
      Serial.println("Sending data to ThingSpeak server...\n");
      String post_data = "field1=";
      post_data = post_data + String(sensorValues[AIR_TEMP1], 1);
      post_data = post_data + "&field2=";
      post_data = post_data + String(sensorValues[AIR_HUM1], 1);
      post_data = post_data + "&field3=";
      post_data = post_data + String(sensorValues[AIR_TEMP2], 1);
      post_data = post_data + "&field4=";
      post_data = post_data + String(sensorValues[AIR_HUM2], 1);
      post_data = post_data + "&field5=";
      post_data = post_data + String(sensorValues[AIR_TEMP3], 1);
      post_data = post_data + "&field6=";
      post_data = post_data + String(sensorValues[AIR_HUM3], 1);
      post_data = post_data + "&field7=";
      post_data = post_data + String(sensorValues[AIR_PRESSURE1], 1);
      post_data = post_data + "&field8=";
      post_data = post_data + String(sensorValues[SUN_LIGHT1], 1);
      Serial.println("Data to be send:");
      Serial.println(post_data);
      client.println("POST /update HTTP/1.1");
      client.println("Host: api.thingspeak.com");
      client.println("Connection: close");
      client.println("X-THINGSPEAKAPIKEY: " + WRITEAPIKEY_1);
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      int thisLength = post_data.length();
      client.println(thisLength);
      client.println();
      client.println(post_data);
      client.println();
      watchdog_reset();
      //delay(1000);
      timer_iot_timeout = millis();
      while ((client.available() == 0) && (millis() < timer_iot_timeout + IOT_TIMEOUT1));
      watchdog_reset();
      timer_iot_timeout = millis();
      while ((millis() < timer_iot_timeout + IOT_TIMEOUT2) && (client.connected()))
      {
        while (client.available() > 0)
        {
          char symb = client.read();
          Serial.print(symb);
          timer_iot_timeout = millis();
        }
      }
      client.stop();
      Serial.println("Packet successfully sent!");
      watchdog_reset();
    }
  }
}

// Send soil temperature data to ThingSpeak
void sendDataIot_ThingSpeak_2()
{
  Serial.print("Connecting to ");
  Serial.print(thingspeak_server);
  Serial.println("...");
  if (client.connect(thingspeak_server, thingspeak_port))
  {
    watchdog_reset();
    if (client.connected())
    {
      watchdog_reset();
      Serial.println("Sending data to ThingSpeak server...\n");
      String post_data = "field1=";
      post_data = post_data + String(sensorValues[SOIL_TEMP1], 1);
      post_data = post_data + "&field2=";
      post_data = post_data + String(sensorValues[SOIL_TEMP2], 1);
      post_data = post_data + "&field3=";
      post_data = post_data + String(sensorValues[SOIL_TEMP3], 1);
      post_data = post_data + "&field4=";
      post_data = post_data + String(sensorValues[SOIL_TEMP4], 1);
      post_data = post_data + "&field5=";
      post_data = post_data + String(sensorValues[SOIL_TEMP5], 1);
      post_data = post_data + "&field6=";
      post_data = post_data + String(sensorValues[SOIL_TEMP6], 1);
      post_data = post_data + "&field7=";
      post_data = post_data + String(sensorValues[SOIL_TEMP7], 1);
      post_data = post_data + "&field8=";
      post_data = post_data + String(sensorValues[SOIL_TEMP8], 1);
      Serial.println("Data to be send:");
      Serial.println(post_data);
      client.println("POST /update HTTP/1.1");
      client.println("Host: api.thingspeak.com");
      client.println("Connection: close");
      client.println("X-THINGSPEAKAPIKEY: " + WRITEAPIKEY_2);
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      int thisLength = post_data.length();
      client.println(thisLength);
      client.println();
      client.println(post_data);
      client.println();
      watchdog_reset();
      //delay(1000);
      timer_iot_timeout = millis();
      while ((client.available() == 0) && (millis() < timer_iot_timeout + IOT_TIMEOUT1));
      watchdog_reset();
      timer_iot_timeout = millis();
      while ((millis() < timer_iot_timeout + IOT_TIMEOUT2) && (client.connected()))
      {
        while (client.available() > 0)
        {
          char symb = client.read();
          Serial.print(symb);
          timer_iot_timeout = millis();
        }
      }
      client.stop();
      Serial.println("Packet successfully sent!");
      watchdog_reset();
    }
  }
}

// Send soil moistures data to ThingSpeak
void sendDataIot_ThingSpeak_3()
{
  Serial.print("Connecting to ");
  Serial.print(thingspeak_server);
  Serial.println("...");
  if (client.connect(thingspeak_server, thingspeak_port))
  {
    watchdog_reset();
    if (client.connected())
    {
      watchdog_reset();
      Serial.println("Sending data to ThingSpeak server...\n");
      String post_data = "field1=";
      post_data = post_data + String(sensorValues[SOIL_MOISTURE1], 1);
      post_data = post_data + "&field2=";
      post_data = post_data + String(sensorValues[SOIL_MOISTURE2], 1);
      post_data = post_data + "&field3=";
      post_data = post_data + String(sensorValues[SOIL_MOISTURE3], 1);
      post_data = post_data + "&field4=";
      post_data = post_data + String(sensorValues[SOIL_MOISTURE4], 1);
      post_data = post_data + "&field5=";
      post_data = post_data + String(sensorValues[SOIL_MOISTURE5], 1);
      post_data = post_data + "&field6=";
      post_data = post_data + String(sensorValues[SOIL_MOISTURE6], 1);
      post_data = post_data + "&field7=";
      post_data = post_data + String(sensorValues[SOIL_MOISTURE7], 1);
      post_data = post_data + "&field8=";
      post_data = post_data + String(sensorValues[SOIL_MOISTURE8], 1);
      Serial.println("Data to be send:");
      Serial.println(post_data);
      client.println("POST /update HTTP/1.1");
      client.println("Host: api.thingspeak.com");
      client.println("Connection: close");
      client.println("X-THINGSPEAKAPIKEY: " + WRITEAPIKEY_3);
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      int thisLength = post_data.length();
      client.println(thisLength);
      client.println();
      client.println(post_data);
      client.println();
      watchdog_reset();
      //delay(1000);
      timer_iot_timeout = millis();
      while ((client.available() == 0) && (millis() < timer_iot_timeout + IOT_TIMEOUT1));
      watchdog_reset();
      timer_iot_timeout = millis();
      while ((millis() < timer_iot_timeout + IOT_TIMEOUT2) && (client.connected()))
      {
        while (client.available() > 0)
        {
          char symb = client.read();
          Serial.print(symb);
          timer_iot_timeout = millis();
        }
      }
      client.stop();
      Serial.println("Packet successfully sent!");
      watchdog_reset();
    }
  }
}

// Send soil moistures data to ThingSpeak
void sendDataIot_ThingSpeak_4()
{
  Serial.print("Connecting to ");
  Serial.print(thingspeak_server);
  Serial.println("...");
  if (client.connect(thingspeak_server, thingspeak_port))
  {
    watchdog_reset();
    if (client.connected())
    {
      watchdog_reset();
      Serial.println("Sending data to ThingSpeak server...\n");
      String post_data = "field1=";
      post_data = post_data + String(controlValues[VALVE_POWER1]);
      post_data = post_data + "&field2=";
      post_data = post_data + String(controlValues[VALVE_POWER2]);
      post_data = post_data + "&field3=";
      post_data = post_data + String(controlValues[WINDOW_STATE1]);
      post_data = post_data + "&field4=";
      post_data = post_data + String(controlValues[LAMP_POWER1]);
      post_data = post_data + "&field5=";
      post_data = post_data + String(sensorValues[DEVICE_TEMP], 1);
      post_data = post_data + "&field6=";
      post_data = post_data + String(sensorValues[MOTION_DETECT], 1);
      post_data = post_data + "&field7=";
      post_data = post_data + String(sensorValues[NETWORK_TIME], 0);
      post_data = post_data + "&field8=";
      post_data = post_data + String(sensorValues[RADIO_COUNTER], 0);
      Serial.println("Data to be send:");
      Serial.println(post_data);
      client.println("POST /update HTTP/1.1");
      client.println("Host: api.thingspeak.com");
      client.println("Connection: close");
      client.println("X-THINGSPEAKAPIKEY: " + WRITEAPIKEY_4);
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      int thisLength = post_data.length();
      client.println(thisLength);
      client.println();
      client.println(post_data);
      client.println();
      watchdog_reset();
      //delay(1000);
      timer_iot_timeout = millis();
      while ((client.available() == 0) && (millis() < timer_iot_timeout + IOT_TIMEOUT1));
      watchdog_reset();
      timer_iot_timeout = millis();
      while ((millis() < timer_iot_timeout + IOT_TIMEOUT2) && (client.connected()))
      {
        while (client.available() > 0)
        {
          char symb = client.read();
          Serial.print(symb);
          timer_iot_timeout = millis();
        }
      }
      client.stop();
      Serial.println("Packet successfully sent!");
      watchdog_reset();
    }
  }
}

// Send magnetic and seismo data to ThingSpeak
void sendDataIot_ThingSpeak_5()
{
  Serial.print("Connecting to ");
  Serial.print(thingspeak_server);
  Serial.println("...");
  if (client.connect(thingspeak_server, thingspeak_port))
  {
    watchdog_reset();
    if (client.connected())
    {
      watchdog_reset();
      Serial.println("Sending data to ThingSpeak server...\n");
      String post_data = "field1=";
      post_data = post_data + String(sensorValues[MAG_X], 1);
      post_data = post_data + "&field2=";
      post_data = post_data + String(sensorValues[MAG_Y], 1);
      post_data = post_data + "&field3=";
      post_data = post_data + String(sensorValues[MAG_Z], 1);
      post_data = post_data + "&field4=";
      post_data = post_data + String(sensorValues[ACC_X], 1);
      post_data = post_data + "&field5=";
      post_data = post_data + String(sensorValues[ACC_Y], 1);
      post_data = post_data + "&field6=";
      post_data = post_data + String(sensorValues[ACC_Z], 1);
      post_data = post_data + "&field7=";
      post_data = post_data + String(sensorValues[GAS_CONC], 1);
      post_data = post_data + "&field8=";
      post_data = post_data + String(sensorValues[WATER_LEVEL], 1);
      Serial.println("Data to be send:");
      Serial.println(post_data);
      client.println("POST /update HTTP/1.1");
      client.println("Host: api.thingspeak.com");
      client.println("Connection: close");
      client.println("X-THINGSPEAKAPIKEY: " + WRITEAPIKEY_5);
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      int thisLength = post_data.length();
      client.println(thisLength);
      client.println();
      client.println(post_data);
      client.println();
      watchdog_reset();
      //delay(1000);
      timer_iot_timeout = millis();
      while ((client.available() == 0) && (millis() < timer_iot_timeout + IOT_TIMEOUT1));
      timer_iot_timeout = millis();
      watchdog_reset();
      while ((millis() < timer_iot_timeout + IOT_TIMEOUT2) && (client.connected()))
      {
        while (client.available() > 0)
        {
          char symb = client.read();
          Serial.print(symb);
          timer_iot_timeout = millis();
        }
      }
      client.stop();
      Serial.println("Packet successfully sent!");
      watchdog_reset();
    }
  }
}

// Send data to ThingWorx and receive control packets
void sendDataIot_ThingWorx_1()
{
  Serial.print("Connecting to ");
  Serial.print(thingworx_address);
  Serial.println("...");
  if (client.connect(thingworx_address, thingworx_port))
  {
    watchdog_reset();
    if (client.connected())
    {
      watchdog_reset();
      Serial.println("Sending data to ThingWorx server...\n");
      Serial.print("POST /Thingworx/Things/");
      client.print("POST /Thingworx/Things/");
      Serial.print(thingworx_thingName);
      client.print(thingworx_thingName);
      Serial.print("/Services/");
      client.print("/Services/");
      Serial.print(thingworx_serviceName);
      client.print(thingworx_serviceName);
      Serial.print("?appKey=");
      client.print("?appKey=");
      Serial.print(thingworx_appKey);
      client.print(thingworx_appKey);
      Serial.print("&method=post&x-thingworx-session=true");
      client.print("&method=post&x-thingworx-session=true");
      watchdog_reset();
      for (int idx = 0; idx < sensorCount; idx ++)
      {
        Serial.print("&");
        client.print("&");
        Serial.print(sensorNames[idx]);
        client.print(sensorNames[idx]);
        Serial.print("=");
        client.print("=");
        Serial.print(sensorValues[idx]);
        client.print(sensorValues[idx]);
      }
      watchdog_reset();
      Serial.println(" HTTP/1.1");
      client.println(" HTTP/1.1");
      Serial.println("Accept: application/json");
      client.println("Accept: application/json");
      Serial.print("Host: ");
      client.print("Host: ");
      Serial.println(thingworx_server);
      client.println(thingworx_server);
      Serial.println("Content-Type: application/json");
      client.println("Content-Type: application/json");
      Serial.println();
      client.println();
      watchdog_reset();
      timer_iot_timeout = millis();
      while ((client.available() == 0) && (millis() < timer_iot_timeout + IOT_TIMEOUT1));
      watchdog_reset();
      int iii = 0;
      bool currentLineIsBlank = true;
      bool flagJSON = false;
      timer_iot_timeout = millis();
      while ((millis() < timer_iot_timeout + IOT_TIMEOUT2) && (client.connected()))
      {
        while (client.available() > 0)
        {
          char symb = client.read();
          Serial.print(symb);
          if (symb == '{')
          {
            flagJSON = true;
          }
          else if (symb == '}')
          {
            flagJSON = false;
          }
          if (flagJSON == true)
          {
            buff[iii] = symb;
            iii ++;
          }
          timer_iot_timeout = millis();
        }
      }
      buff[iii] = '}';
      buff[iii + 1] = '\0';
      Serial.println(buff);
      client.stop();
      watchdog_reset();
      StaticJsonBuffer<BUFF_LENGTH> jsonBuffer;
      JsonObject& json_array = jsonBuffer.parseObject(buff);
      valve_control1 = json_array["valve_control1"];
      valve_control2 = json_array["valve_control2"];
      window_control1 = json_array["window_control1"];
      lamps_control1 = json_array["lamps_control1"];
      Serial.println("Input valve state: " + String(valve_control1));
      Serial.println("Output valve state: " + String(valve_control2));
      Serial.println("Window state: " + String(window_control1));
      Serial.println("Lamps state: " + String(lamps_control1));
      Serial.println();
      // Control devices (only when state changes)
      // Input valve
      if (valve_control1 != old_valve_control1)
      {
        Serial.println("Input valve state has been changed");
        if (valve_control1)
        {
          controlTimers[VALVE_POWER1] = 3600;
          sensorValues[VALVE_TIMER1] = controlTimers[VALVE_POWER1];
          sensorFlags[VALVE_TIMER1] = true;
        }
        else
        {
          controlTimers[VALVE_POWER1] = 0;
          sensorValues[VALVE_TIMER1] = controlTimers[VALVE_POWER1];
          sensorFlags[VALVE_TIMER1] = false;
        }
      }
      // Output valve
      if (valve_control2 != old_valve_control2)
      {
        Serial.println("Output valve state has been changed");
        if (valve_control2)
        {
          controlTimers[VALVE_POWER2] = 3600;
          sensorValues[VALVE_TIMER2] = controlTimers[VALVE_POWER2];
          sensorFlags[VALVE_TIMER2] = true;
        }
        else
        {
          controlTimers[VALVE_POWER2] = 0;
          sensorValues[VALVE_TIMER2] = controlTimers[VALVE_POWER2];
          sensorFlags[VALVE_TIMER2] = false;
        }
      }
      // Window
      if (window_control1 != old_window_control1)
      {
        Serial.println("Window state has been changed");
        if (window_control1)
        {
          controlTimers[WINDOW_STATE1] = 3600;
          sensorValues[WINDOW_TIMER1] = controlTimers[WINDOW_STATE1];
          sensorFlags[WINDOW_TIMER1] = true;
        }
        else
        {
          controlTimers[WINDOW_STATE1] = 0;
          sensorValues[WINDOW_TIMER1] = controlTimers[WINDOW_STATE1];
          sensorFlags[WINDOW_TIMER1] = false;
        }
        digitalWrite(RELAY_PIN5, LOW);
      }
      else
      {
        digitalWrite(RELAY_PIN5, HIGH);
      }
      // Lamps
      if (lamps_control1 != old_lamps_control1)
      {
        Serial.println("Lamps state has been changed");
        if (lamps_control1)
        {
          controlTimers[LAMP_POWER1] = 3600;
          sensorValues[LAMPS_TIMER1] = controlTimers[LAMP_POWER1];
          sensorFlags[LAMPS_TIMER1] = true;
        }
        else
        {
          controlTimers[LAMP_POWER1] = 0;
          sensorValues[LAMPS_TIMER1] = controlTimers[LAMP_POWER1];
          sensorFlags[LAMPS_TIMER1] = false;
        }
      }
      old_valve_control1 = valve_control1;
      old_valve_control2 = valve_control2;
      old_window_control1 = window_control1;
      old_lamps_control1 = lamps_control1;
      Serial.println("Packet successfully sent!");
      Serial.println();
    }
  }
}

// Automatic function control
void controlDevices_1()
{
  // Decrement control timers and automatic off switched-on devices
  for (int u = 0; u < controlCount; u++)
  {
    controlTimers[u] = controlTimers[u] - 1;
    if (controlTimers[u] <= 0)
    {
      controlTimers[u] = 0;
      controlValues[u] = false;
    }
    else
    {
      controlValues[u] = true;
      controlFlags[u] = true;
    }
  }
  watchdog_reset();

  // Copy timers to sensors values array
  sensorValues[VALVE_TIMER1] = controlTimers[VALVE_POWER1];
  sensorValues[VALVE_TIMER2] = controlTimers[VALVE_POWER2];
  sensorValues[WINDOW_TIMER1] = controlTimers[WINDOW_STATE1];
  sensorValues[LAMPS_TIMER1] = controlTimers[LAMP_POWER1];

  // Read control values and power devices
  digitalWrite(RELAY_PIN1, controlValues[VALVE_POWER1]);
  digitalWrite(RELAY_PIN2, controlValues[VALVE_POWER2]);
  digitalWrite(RELAY_PIN3, controlValues[LAMP_POWER1]);
  if (controlValues[WINDOW_STATE1])
  {
    servo_1.write(180);
  }
  else
  {
    servo_1.write(0);
  }
}

// Reset software watchdog timer
void watchdog_reset()
{
  wdt_timer = 0;
}

#define SOIL_TEMP1     0
#define SOIL_TEMP2     1
#define SOIL_TEMP3     2
#define SOIL_TEMP4     3
#define SOIL_TEMP5     4
#define SOIL_TEMP6     5
#define SOIL_TEMP7     6
#define SOIL_TEMP8     7
#define SOIL_TEMP9     8
#define SOIL_MOISTURE1 9
#define SOIL_MOISTURE2 10
#define SOIL_MOISTURE3 11
#define SOIL_MOISTURE4 12
#define SOIL_MOISTURE5 13
#define SOIL_MOISTURE6 14
#define SOIL_MOISTURE7 15
#define SOIL_MOISTURE8 16
#define SOIL_MOISTURE9 17
#define AIR_TEMP1      18
#define AIR_TEMP2      19
#define AIR_TEMP3      20
#define AIR_HUM1       21
#define AIR_HUM2       22
#define AIR_HUM3       23
#define AIR_PRESSURE1  24
#define SUN_LIGHT1     25
#define MAG_X          26
#define MAG_Y          27
#define MAG_Z          28
#define ACC_X          29
#define ACC_Y          30
#define ACC_Z          31
#define GYR_X          32
#define GYR_Y          33
#define GYR_Z          34
#define DEVICE_TEMP    35
#define GAS_CONC       36
#define MOTION_DETECT  37
#define VALVE_TIMER1   38
#define VALVE_TIMER2   39
#define WINDOW_TIMER1  40
#define LAMPS_TIMER1   41
#define NETWORK_TIME   42
#define RADIO_COUNTER  43
#define WATER_LEVEL    44

// Interrupt service routine for timer3 overflow
ISR(TIMER3_OVF_vect)
{
  wdt_timer = wdt_timer + 1;
  if (wdt_timer > MAX_WDT)
  {
    wdt_timer = 0;
    TIMSK3 = 0;
    TCNT3 = 0;
    TCCR3A = 0;
    TCCR3B = 0;
    Serial.println("WDT! Resetting...\n");
    delay(1000);
    pinMode(HRESETPIN, OUTPUT);
    digitalWrite(HRESETPIN, LOW);
  }
  TCNT3 = timer3_counter;
}
