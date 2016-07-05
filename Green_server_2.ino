#include <DHT.h>
#include <L3G.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <VirtualWire.h>
#include <UIPEthernet.h>
#include <BH1750FVI.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>

// LED pin
#define LED_PIN      13

// Relay outputs
#define RELAY_PIN1   22
#define RELAY_PIN2   23
#define RELAY_PIN3   24
#define RELAY_PIN4   25

// Motor outputs
#define PWMA 4
#define DIRA 5
#define PWMB 6
#define DIRB 7

// 433 MHz receiver parameters and vars
#define RECEIVER_PIN 2

// DHT11 and DHT22 sensors
#define DHT11_PIN1 8
#define DHT11_PIN2 9
#define DHT22_PIN1 10
DHT dht11_1(DHT11_PIN1, DHT11);
DHT dht11_2(DHT11_PIN2, DHT11);
DHT dht22_1(DHT22_PIN1, DHT22);

// LCD display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Period of print data to LCD display
#define LCD_UPDATE_TIME 15000

// Period of print data to terminal
#define TERM_UPDATE_TIME 60000

// Period of read BH1750 light sensor
#define LIGHT_UPDATE_TIME 60000

// Period of read DHT11 and DHT22 sensors
#define DHT_UPDATE_TIME 60000

// Period of read 433 MHz receiver
#define REC_UPDATE_TIME 50

// Period of reset flags for radio sensors
#define RADIORESET_UPDATE_TIME 600000

// Period of send data to IoT server
#define IOT_UPDATE_TIME 300000

// Period of read BMP085 pressure sensors
#define PRESS_UPDATE_TIME 60000

// Timer for DHT11 and DHT22 sensors
long timer_dht = 0;

// Timer for BH1750 light sensor
long timer_light = 0;

// Timer for BMP085 pressure sensor
long timer_press = 0;

// Timer for LCD display
long timer_lcd = 0;

// Timer for terminal print
long timer_term = 0;

// Timer for IoT server
long timer_iot = 0;

// Timer for 433 MHz receiver
long timer_rec = 0;

// Timer for reset flags for radio sensors
long timer_radioreset = 0;

// IoT server network parameters
char iot_server[] = "cttit5402.cloud.thingworx.com";      // Name address for Google (using DNS)
IPAddress iot_address(52, 87, 101, 142);
char appKey[] = "dacefda9-bca7-47bb-b2dd-c57c7651749d";   // APP access key for ThingWorx
char thingName[] = "SmartGreen";                          // Name of your Thing in ThingWorx
char serviceName[] = "setAll";                            // Name of your Service (see above)

// IoT server sensor parameters
#define sensorCount 26                                    // How many values you will be pushing to ThingWorx
char* sensorNames[] = {"soil_temp1", "soil_temp2", "soil_temp3", "soil_temp4", "soil_temp5", "soil_temp6", "soil_temp7", "soil_temp8", "soil_temp9"
                       , "soil_moisture1", "soil_moisture2", "soil_moisture3", "soil_moisture4", "soil_moisture5", "soil_moisture6", "soil_moisture7", "soil_moisture8", "soil_moisture9"
                       , "air_temp1", "air_temp2", "air_temp3", "air_hum1", "air_hum2", "air_hum3", "air_pressure1", "sun_light1"
                      };
float sensorValues[sensorCount] = {0};
#define SOIL_TEMP1 0
#define SOIL_TEMP2 1
#define SOIL_TEMP3 2
#define SOIL_TEMP4 3
#define SOIL_TEMP5 4
#define SOIL_TEMP6 5
#define SOIL_TEMP7 6
#define SOIL_TEMP8 7
#define SOIL_TEMP9 8
#define SOIL_MOISTURE1 9
#define SOIL_MOISTURE2 10
#define SOIL_MOISTURE3 11
#define SOIL_MOISTURE4 12
#define SOIL_MOISTURE5 13
#define SOIL_MOISTURE6 14
#define SOIL_MOISTURE7 15
#define SOIL_MOISTURE8 16
#define SOIL_MOISTURE9 17
#define AIR_TEMP1 18
#define AIR_TEMP2 19
#define AIR_TEMP3 20
#define AIR_HUM1 21
#define AIR_HUM2 22
#define AIR_HUM3 23
#define AIR_PRESSURE1 24
#define SUN_LIGHT1 25

// Read flags for radio sensors
uint8_t sensorFlags[sensorCount] = {0};

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
  Serial.begin(9600);
  while (!Serial) {}

  // Init onboard LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Init relay outputs
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);
  pinMode(RELAY_PIN3, OUTPUT);
  pinMode(RELAY_PIN4, OUTPUT);

  // Init motor outputs
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Init receiver
  vw_set_rx_pin(RECEIVER_PIN);
  vw_setup(1024);
  vw_rx_start();

  // Init DHT11 and DHT22 sensors
  dht11_1.begin();
  dht11_2.begin();
  dht22_1.begin();

  // Init BH1750 light sensor
  LightSensor_1.begin();
  LightSensor_1.setMode(Continuously_High_Resolution_Mode);

  // Init BMP085 sensor
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP085 sensor!");
  }
  // Init L3G4200D sensor
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
  }
  else
  {
    gyro.enableDefault();
  }
  // Init HMC5883L sensor
  if (!mag.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor!");
  }
  // Init ADXL345 sensor
  if (!accel.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor!");
  }
  else
  {
    accel.setRange(ADXL345_RANGE_16_G);
    accel.setDataRate(ADXL345_DATARATE_200_HZ);
  }

  // Init LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Init Ethernet ENC28J60
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip, dnsServerIP, gateway, subnet);
  }
  Serial.print("LocalIP:\t\t");
  Serial.println(Ethernet.localIP());
  Serial.print("SubnetMask:\t\t");
  Serial.println(Ethernet.subnetMask());
  Serial.print("GatewayIP:\t\t");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("dnsServerIP:\t\t");
  Serial.println(Ethernet.dnsServerIP());
  Serial.println("");

  // One time read all sensors and print data
  readSensorDHT11_1();
  readSensorDHT11_2();
  readSensorDHT22_1();
  readSensorBH1750_1();
  readSensorBMP085_1();
  lcd.clear();
  lcd.setCursor(0, 0); lcd_printstr("OUT_T = " + String(sensorValues[AIR_TEMP3], 1) + " *C");
  lcd.setCursor(0, 1); lcd_printstr("OUT_H = " + String(sensorValues[AIR_HUM3], 1) + " %");
  lcd.setCursor(0, 2); lcd_printstr("PRESS = " + String(sensorValues[AIR_PRESSURE1], 1) + " mm");
  lcd.setCursor(0, 3); lcd_printstr("LIGHT = " + String(sensorValues[SUN_LIGHT1], 1) + " LX");
  printSerialData();
}

void loop()
{
  // Read DHT11 and DHT22 senors
  if (millis() > timer_dht + DHT_UPDATE_TIME)
  {
    // Read sensors
    readSensorDHT11_1();
    readSensorDHT11_2();
    readSensorDHT22_1();
    // Reset timer
    timer_dht = millis();
  }

  // Read BH1750 light sensor
  if (millis() > timer_light + LIGHT_UPDATE_TIME)
  {
    // Read sensor
    readSensorBH1750_1();
    // Reset timer
    timer_light = millis();
  }

  // Read BMP085 pressure sensor
  if (millis() > timer_press + PRESS_UPDATE_TIME)
  {
    // Read sensor
    readSensorBMP085_1();
    // Reset timer
    timer_press = millis();
  }

  // Reset flags of radio sensors
  if (millis() > timer_radioreset + RADIORESET_UPDATE_TIME)
  {
    // Reset flags
    for (int u = 0; u < sensorCount; u++)
    {
      sensorFlags[u] = false;
    }
    // Reset timer
    timer_radioreset = millis();
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
      digitalWrite(LED_PIN, LOW);
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
      if ((sensor_addr >= 1) && (sensor_addr <= 9))
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
      }
    }
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
        break;
      case 1:
        lcd.clear();
        lcd.setCursor(0, 0); lcd_printstr("OUT_T = " + String(sensorValues[AIR_TEMP3], 1) + " *C");
        lcd.setCursor(0, 1); lcd_printstr("OUT_H = " + String(sensorValues[AIR_HUM3], 1) + " %");
        lcd.setCursor(0, 2); lcd_printstr("PRESS = " + String(sensorValues[AIR_PRESSURE1], 1) + " mm");
        lcd.setCursor(0, 3); lcd_printstr("LIGHT = " + String(sensorValues[SUN_LIGHT1], 1) + " LX");
        break;
      case 2:
        lcd.clear();
        if (sensorFlags[SOIL_TEMP1])
        {
          lcd.setCursor(0, 0); lcd_printstr("R_T1 = " + String(sensorValues[SOIL_TEMP1], 1) + " *C");
        }
        else
        {
          lcd.setCursor(0, 0); lcd_printstr("R_T1 = NO DATA");
        }
        if (sensorFlags[SOIL_MOISTURE1])
        {
          lcd.setCursor(0, 1); lcd_printstr("R_M1 = " + String(sensorValues[SOIL_MOISTURE1], 1) + " %");
        }
        else
        {
          lcd.setCursor(0, 1); lcd_printstr("R_M1 = NO DATA");
        }
        if (sensorFlags[SOIL_TEMP2])
        {
          lcd.setCursor(0, 2); lcd_printstr("R_T2 = " + String(sensorValues[SOIL_TEMP2], 1) + " *C");
        }
        else
        {
          lcd.setCursor(0, 2); lcd_printstr("R_T2 = NO DATA");
        }
        if (sensorFlags[SOIL_MOISTURE2])
        {
          lcd.setCursor(0, 3); lcd_printstr("R_M2 = " + String(sensorValues[SOIL_MOISTURE2], 1) + " %");
        }
        else
        {
          lcd.setCursor(0, 3); lcd_printstr("R_M2 = NO DATA");
        }
        break;
      case 3:
        lcd.clear();
        //lcd.setCursor(0, 0); lcd_printstr("R_T3 = " + String(sensorValues[SOIL_TEMP3], 1) + " *C");
        //lcd.setCursor(0, 1); lcd_printstr("R_M3 = " + String(sensorValues[SOIL_MOISTURE3], 1) + " %");
        //lcd.setCursor(0, 2); lcd_printstr("R_T4 = " + String(sensorValues[SOIL_TEMP4], 1) + " *C");
        //lcd.setCursor(0, 3); lcd_printstr("R_H4 = " + String(sensorValues[SOIL_MOISTURE4], 1) + " %");
        break;
      default:
        break;
    }
    // Change page number
    page_number++;
    if (page_number > 2)
    {
      page_number = 0;
    }
    // Reset timer
    timer_lcd = millis();
  }

  // Print data to terminal
  if (millis() > timer_term + TERM_UPDATE_TIME)
  {
    // Print all data to serial terminal
    printSerialData();
    // Reset timer
    timer_term = millis();
  }

  // Send data to IoT ThingSpeak
  if (millis() > timer_iot + IOT_UPDATE_TIME)
  {
    // Send air sensors data
    sendDataIot_ThingSpeak_1();
    // Send soil temperature sensors data
    sendDataIot_ThingSpeak_2();
    // Reset timer
    timer_iot = millis();
  }
}

// Read DHT11 sensor #1
void readSensorDHT11_1()
{
  sensorValues[AIR_HUM1] = dht11_1.readHumidity();
  sensorValues[AIR_TEMP1] = dht11_1.readTemperature();
}

// Read DHT11 sensor #2
void readSensorDHT11_2()
{
  sensorValues[AIR_HUM2] = dht11_2.readHumidity();
  sensorValues[AIR_TEMP2] = dht11_2.readTemperature();
}

// Read DHT22 sensor #1
void readSensorDHT22_1()
{
  sensorValues[AIR_HUM3] = dht22_1.readHumidity();
  sensorValues[AIR_TEMP3] = dht22_1.readTemperature();
}

// Read BH1750 light sensor #1
void readSensorBH1750_1()
{
  sensorValues[SUN_LIGHT1] = LightSensor_1.getAmbientLight();
}

// Read BMP085 air pressure sensor #1
void readSensorBMP085_1()
{
  sensors_event_t p_event;
  bmp.getEvent(&p_event);
  if (p_event.pressure)
  {
    sensorValues[AIR_PRESSURE1] = p_event.pressure * 7.5006 / 10;
    //bmp.getTemperature(&t);
  }
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
    if (client.connected())
    {
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
      //delay(1000);
      timer_iot_timeout = millis();
      while ((client.available() == 0) && (millis() < timer_iot_timeout + IOT_TIMEOUT1));
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
    if (client.connected())
    {
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
      //delay(1000);
      timer_iot_timeout = millis();
      while ((client.available() == 0) && (millis() < timer_iot_timeout + IOT_TIMEOUT1));
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
    }
  }
}
