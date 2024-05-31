// Need to install these libraries
#include <DFRobot_BMP3XX.h>   // by DFRobot
#include <DFRobot_LIS2DW12.h> // by DFRobot
#include "CRC32.h"            //CRC by Rob Tillart

// #define DEBUG // Uncomment this line for debug
// When debug on, loop time >20 ms because of serial prints
// When debug off, loop takes ~6 ms so ok

// Declare library objects
CRC32 crc;
DFRobot_BMP388_I2C pressure_sensor;
DFRobot_LIS2DW12_I2C acceleration_sensor;

// Define i/o pinouts
const int inc_enc_a = 2;
const int inc_enc_b = 3;
const int abs_enc_a = 4;
const int abs_enc_b = 5;
const int abs_enc_c = 6;
const int abs_enc_d = 7;
const int green_led = 8;
const int red_led = 9;
const int humidity_analog = A0;

// Define packet structure
typedef struct {
  uint8_t start;
  uint32_t altitude; 
  uint32_t acceleration;
  uint32_t temperature;
  uint32_t humidity;
  uint32_t wind_spd;
  uint8_t wind_dir;  
  uint32_t checksum;  
} packet_t;
packet_t packet;

// Global variables
int inc_enc_count = 0;
unsigned long loop_time;
const int loop_period = 20;
bool led_enable = true;
uint8_t error_code = 0;

// ERROR CODE: //
// 0000 -> no error
// 0001 -> temperature error
// 0010 -> humidity error
// 0100 -> i2c error (i.e. accelerometer/barometer)

void setup(void)
{
  // Begin serial communications
  Serial.begin(38400);

  // +++++++++++++++ Initialise BMP388 pressure sensor +++++++++++++++ //
  int err;
  while( ERR_OK != (err = pressure_sensor.begin()) ){
    if(ERR_DATA_BUS == err){
      Serial.println("Data bus error!!!");
    }else if(ERR_IC_VERSION == err){
      Serial.println("Chip versions do not match!!!");
    }
    delay(1000);
  }
  // Set sampling mode
  while( !pressure_sensor.setSamplingMode(pressure_sensor.eNormalPrecision2) ){
    Serial.println("Set samping mode fail, retrying....");
    delay(1000);
  }
  // Calibrate absolute
  delay(100);
  if(!pressure_sensor.calibratedAbsoluteDifference(0) ){
    Serial.println("Unsuccessful in calibrating absolute pressure");
  }
  delay(1000);

  // +++++++++++++++ Initialise LIS2DW12 acceleration sensor +++++++++++++++ //
  while(!acceleration_sensor.begin()){
     Serial.println("Communication failed, check the connection and I2C address setting when using I2C communication.");
     delay(1000);
  }
  acceleration_sensor.softReset(); // soft reset
  acceleration_sensor.continRefresh(true); // continuously collect date
  // Set data rate, filtering, and dynamic range
  acceleration_sensor.setDataRate(DFRobot_LIS2DW12::eRate_50hz);
  acceleration_sensor.setRange(DFRobot_LIS2DW12::e2_g);
  acceleration_sensor.setFilterPath(DFRobot_LIS2DW12::eLPF);
  acceleration_sensor.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  acceleration_sensor.setPowerMode(DFRobot_LIS2DW12::eHighPerformance_14bit);
  delay(100);

  // +++++++++++++++ Initialise incremental encoder +++++++++++++++ //
  pinMode(inc_enc_a, INPUT);
  pinMode(inc_enc_b, INPUT);
  attachInterrupt(digitalPinToInterrupt(inc_enc_b), increment, RISING);

  // +++++++++++++++ Initialise absolute encoder +++++++++++++++ //
  pinMode(abs_enc_a, INPUT);
  pinMode(abs_enc_b, INPUT);
  pinMode(abs_enc_c, INPUT);
  pinMode(abs_enc_d, INPUT);

  // +++++++++++++++ Initialise LEDs +++++++++++++++ //
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  digitalWrite(red_led, HIGH);
  digitalWrite(green_led, LOW);
}


void loop()
{
  // Run once every 0.02s (50 Hz)
  unsigned long current_time = millis();
  if (current_time - loop_time < loop_period) {
    return;
  }
  loop_time = current_time;

  error_code = 0;

  // Toggle LED based on serial command
  while (Serial.available()){
    if (Serial.read() == 0xFF){
      led_enable = true;
      digitalWrite(green_led, LOW);
    } else if (Serial.read() == 0x00) {
      led_enable = false;
      digitalWrite(red_led, HIGH);
      digitalWrite(green_led, HIGH);
    }
  }
  digitalWrite(red_led, HIGH);
  
  // Get sensor values 
  float altitude = pressure_sensor.readAltitudeM();
  float acceleration = acceleration_sensor.readAccZ() - 1000;
  float temperature = pressure_sensor.readTempC();
  float humidity = humidity_calc(temperature, humidity_read(humidity_analog));
  
  // Update wind speed and reset counter value
  float wind_spd = float(inc_enc_count) / 0.24 / loop_period * (2*PI*0.04);  // 0.04 m radius
  inc_enc_count = 0;  

  // Update wind direction, converting gray to binary
  uint8_t wind_dir = 0;  
  wind_dir = gray_convert(digitalRead(abs_enc_a), digitalRead(abs_enc_b), digitalRead(abs_enc_c), digitalRead(abs_enc_d));
  wind_dir |= error_code << 4; // Pack the error code into here

  // Assemble packet and calculate checksum
  packet.start = 0xFF;
  memcpy(&(packet.altitude), &altitude, sizeof wind_spd);
  memcpy(&(packet.acceleration), &acceleration, sizeof wind_spd);
  memcpy(&(packet.temperature), &temperature, sizeof wind_spd);
  memcpy(&(packet.humidity), &humidity, sizeof wind_spd);
  memcpy(&(packet.wind_spd), &wind_spd, sizeof wind_spd);
  memcpy(&(packet.wind_dir), &wind_dir, sizeof wind_spd);
  crc.restart();
  crc.add((uint8_t*) &packet, 22);
  packet.checksum = crc.calc();

  // Write packet onto serial link
  Serial.write((uint8_t *) &packet, sizeof(packet));

  #ifdef DEBUG
    Serial.println();
    Serial.print("Packet : ");
    uint8_t* packet_arr = (uint8_t*) &packet;
    for (int i=0; i<22; i++) {
      Serial.print((int) packet_arr[i],HEX);
    }
    Serial.println();  

    Serial.print("Altitude : ");
    Serial.print(altitude);
    Serial.println(" m");

    Serial.print("Acceleration: ");
    Serial.print(acceleration);
    Serial.println(" mg");

    Serial.print("Temperature : ");
    Serial.print(temperature);
    Serial.println(" C");

    Serial.print("Humidity: ");
    Serial.println(humidity);

    Serial.print("Wind Speed: ");
    Serial.print(wind_spd);
    Serial.println(" m/s");

    Serial.print("Wind Direction: ");
    Serial.println(wind_dir);

    Serial.print("CRC Checksum: ");
    Serial.println(packet.checksum, HEX);

    Serial.println();
  #endif
}    

// Interrupt function to increment encoder counter
void increment(){
  inc_enc_count++;
}

// Helper function to map gray code to binary
uint8_t gray_convert(bool a, bool b, bool c, bool d){
  int gray = 0;
  gray |= a << 3;
  gray |= b << 2;
  gray |= c << 1;
  gray |= d;
  switch(gray) {
    case 0b0000: return 0;
    case 0b0001: return 1;
    case 0b0011: return 2;
    case 0b0010: return 3;
    case 0b0110: return 4;
    case 0b0111: return 5;
    case 0b0101: return 6;
    case 0b0100: return 7;
    case 0b1100: return 8;
    case 0b1101: return 9;
    case 0b1111: return 10;
    case 0b1110: return 11;
    case 0b1010: return 12;
    case 0b1011: return 13;
    case 0b1001: return 14;
    case 0b1000: return 15;
    default:
      if (led_enable) {
        error_code = 0b0100;
        digitalWrite(red_led,LOW);
      }
      return 0;
  }
}

// Humidity helper functions
float humidity_table[15][11] = {
{34100, 24000, 17800, 13000, 9210, 6570, 4910, 3400, 2220, 1620, 1080},
{12600, 9000, 6670, 4870, 3150, 2550, 1900, 1350, 920, 680, 470},
{5160, 3530, 2750, 2010, 1480, 1090, 810, 590, 420, 310, 230},
{2270, 1600, 1230, 910, 670, 500, 380, 280, 200, 150, 120},
{1060, 770, 590, 440, 330, 250, 190, 140, 110, 81.4, 62.3},
{530, 390, 300, 220, 170, 130, 100, 76.3, 58.8, 45.5, 35.6},
{270, 210, 160, 120, 92.8, 71.8, 55.6, 43.3, 34.0, 26.7, 21.3},
{150, 110, 87.5, 67.7, 52.7, 41.3, 32.4, 25.6, 20.5, 16.4, 13.3},
{82.9, 64.2, 50.0, 39.3, 31.0, 24.7, 19.7, 15.9, 12.8, 10.4, 8.48},
{47.4, 37.2, 29.4, 23.5, 18.8, 15.1, 12.2, 10.0, 8.18, 6.74, 5.60},
{27.9, 22.2, 17.8, 14.4, 11.7, 9.53, 7.81, 6.48, 5.38, 4.50, 3.78},
{16.8, 13.5, 11.0, 9.06, 7.45, 6.16, 5.13, 4.32, 3.63, 3.07, 2.61},
{10.4, 8.37, 6.91, 5.82, 4.85, 4.08, 3.44, 2.94, 2.50, 2.14, 1.83},
{6.51, 5.31, 4.45, 3.82, 3.24, 2.76, 2.36, 2.05, 1.76, 1.52, 1.31},
{4.17, 3.44, 2.93, 2.57, 2.21, 1.92, 1.66, 1.46, 1.26, 1.11, 0.96}
};

float humidity_calc(float temperature, int analog){
  //return zero if temperature or analog reading out of range
  if (temperature > 55 || temperature < 5) {
    error_code = 0b0001;
    if (led_enable) {
      digitalWrite(red_led,LOW);
    }
    return 0;  
  }
  //calculate sensor resistance
  float voltage = 5.0/1024.0 * float(analog); // with a 5V 10-bit ADC
  float resistance = 5000.0/voltage - 1000.0; // using 1 Mohm resistor
  //interpolate by temperature
  float humidity_row[11] = {};
  for (int i=0; i<11; i++) {
    int lower = floor((temperature-5)/5);
    int upper = ceil((temperature-5)/5);
    humidity_row[i] = (upper*5.0+5.0 - temperature) / 5.0 * (humidity_table[i][lower]-humidity_table[i][upper]) + humidity_table[i][upper];
  }
  //interpolate by resistance
  float humidity_val = 0;
  //if (resistance > humidity_row[0] || resistance < humidity_row[10]){
  if (voltage > 4.8){
    error_code = 0b0010;
    if (led_enable) {
      digitalWrite(red_led,LOW);
    }
  }
  for (int i=0; i<10; i++){
    if (resistance < humidity_row[i] && resistance > humidity_row[i+1]) {
      humidity_val = ((humidity_row[i] - resistance) / (humidity_row[i] - humidity_row[i+1]))*5 + 20 + i*5;
    }
  }
  return humidity_val;
}

int humidity_read (int pin){
  int analog = 0;
  analog += analogRead(pin);
  delay(1);
  analog += analogRead(pin);
  delay(1);
  analog += analogRead(pin);
  return analog/3;
}
