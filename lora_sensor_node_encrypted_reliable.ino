// Arduino libraries
#include <Arduino.h>
#include <Wire.h>               // I2C <-> sensors & RTC (peripherals)
#include <SPI.h>                // SPI <-> LoRa module

// Peripherals libraries
#include <Adafruit_SHT31.h>     // Temperature & humidity sensor
#include <Adafruit_ADS1015.h>   // ADC <-> wind speed sensor
#include <MAX44009.h>           // Light intensity sensor
#include <RtcDS3231.h>          // RTC for periodic wakeup

// LoRa libraries
#include <RH_RF95.h>            // Low level LoRa radio driver
#include <RHReliableDatagram.h> // Driver manager for reliability
#include <AES.h>                // For use with AES128 cipher

// Define used pins
#define WAKEUP_PIN         GPIO_NUM_13
#define WIND_SENSOR_EN_PIN GPIO_NUM_25
#define CHARGING_EN_PIN    GPIO_NUM_12

// Define addresses of peripherals
#define SHT31_ADDR       (uint8_t) 0x44 // ADDR pin pulled low
#define ADS1115_ADDR     (uint8_t) 0x48
#define MAX44009_ADDR    (uint8_t) 0x4A
// Used DS3231 default I2C address (defined in the RTC library)
#define WIND_SENSOR_ADDR 0 // Wind sensor <-> AIN0
#define BATT_ADDR        1 // Battery <-> AIN1

// Define LoRa parameters
#define GATEWAY_ADDR 1
#define SENSOR_ADDR  2
#define FREQ         915.0
// #define MODEM_CONFIG RH_RF95::Bw125Cr48Sf4096
#define MODEM_CONFIG RH_RF95::Bw125Cr45Sf128
#define TX_POWER     17
#define SS_PIN       GPIO_NUM_18
#define IRQ_PIN      GPIO_NUM_26

// Instantiate peripherals objects
Adafruit_SHT31     sht31 = Adafruit_SHT31();
Adafruit_ADS1115   ads1115(ADS1115_ADDR);
MAX44009           max44009(MAX44009_ADDR);
RtcDS3231<TwoWire> ds3231(Wire);

// Instantiate LoRa objects
RH_RF95 rf95(SS_PIN, IRQ_PIN);
RHReliableDatagram manager(rf95, SENSOR_ADDR);

// RTC variables (preserve its values in deep sleep mode)
RTC_DATA_ATTR int readCount = 0;
RTC_DATA_ATTR float tempMax = 0.0;
RTC_DATA_ATTR float tempMin = 0.0;
RTC_DATA_ATTR float humMax = 0.0;
RTC_DATA_ATTR float humMin = 0.0;
RTC_DATA_ATTR float windMax = 0.0;
RTC_DATA_ATTR float windMin = 0.0;
RTC_DATA_ATTR float energyAccum = 0.0;
RTC_DATA_ATTR float sunshineAccum = 0.0;

void setup() 
{
  Serial.begin(115200);
  Serial.println("WOKE UP! Starting program.");
  
  // Enable external wakeup sources when input level is LOW
  esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 0);
  
  // Turn on wind speed sensor (active high)
  pinMode(WIND_SENSOR_EN_PIN, OUTPUT);
  digitalWrite(WIND_SENSOR_EN_PIN, HIGH);
  
  // Disable battery charging (active low)
  pinMode(CHARGING_EN_PIN, OUTPUT);
  digitalWrite(CHARGING_EN_PIN, HIGH);
  
  // Initialize peripherals
  Wire.begin(4, 15);
  sht31.begin(SHT31_ADDR);
  max44009.Begin(0, 188000); // full range
  ads1115.begin();
  rtcInit();
  
  // Initialize LoRa
  LoRaInit();

  delay(500);
  
  // Read current date & time from RTC
  RtcDateTime timestamp = ds3231.GetDateTime();
  
  Serial.printf("Timestamp: %d/%d/%d %d:%d:%d\n", timestamp.Year(), 
                timestamp.Month(), timestamp.Day(), timestamp.Hour(), 
                timestamp.Minute(), timestamp.Second());
  Serial.println("Getting measurements from sensors ... ");
  
  // Get current sensor readings
  float temp, hum, energy, wind;
  getSensorReadings(&temp, &hum, &energy, &wind, &sunshineAccum);
  
  // Increment readCount on every new measurement
  ++readCount;
  
  Serial.printf("Temp. (*C) = %.3f\n", temp); // 0.015 *C resolution
  Serial.printf("Hum. (Percent) = %.2f\n", hum); // 0.01 % resolution
  Serial.printf("Energy (Joule/m2) = %.3f\n", energy);
  Serial.printf("Wind Speed (m/s) = %.1f\n", wind); //0.1 m/s resolution
  
  // Accumulate sensor readings over 1 hour
  accumReadings(temp, hum, energy, wind);
  
  // Send readings at an interval
  int isSendingTime = (timestamp.Minute() % 2 == 0) ? 1 : 0;
  
  if (isSendingTime)
  {
    // Calculate hourly average weather condition
    float tempMean, humMean, energy1Hour, windMean, sunshineHour; 
    hourlyAvg(&tempMean, &humMean, &energy1Hour, &windMean, &sunshineHour);
    
    // Read current battery voltage
    int16_t adcReading = ads1115.readADC_SingleEnded(BATT_ADDR);
    float vBattery = (adcReading * 0.0001875);
    
    Serial.printf("Battery = %.3f\n", vBattery);
    
    // Compose message payload to be sent to gateway
    uint8_t payload[31];
    composePayload(payload, timestamp, vBattery, tempMean, humMean, energy1Hour, windMean, sunshineHour);
    
    // Send hourly average readings to gateway
    sendReadings(payload, sizeof(payload));
    
    Serial.println("Readings sent to gateway.");
    
    // Reset accumulated readings over 1 hour
    tempMean = 0.0;
    humMean = 0.0;
    energyAccum = 0.0;
    windMean = 0.0;
    sunshineAccum = 0.0;
    
    // Reset readCount
    readCount = 0;
  }
  
  Serial.println("DONE! Going to deep sleep now.");
  
  // Turn off wind sensor and enable battery charging before deep sleep
  digitalWrite(WIND_SENSOR_EN_PIN, LOW);
  digitalWrite(CHARGING_EN_PIN, LOW);
  
  // Set LoRa module to sleep mode
  rf95.sleep();
  
  // Go to deep sleep mode
  esp_deep_sleep_start();
}

void loop()
{
  // Do nothing
}

void rtcInit()
{  
  // Begin I2C communication
  ds3231.Begin();
  // Disable 32 kHz pin (not used)
  ds3231.Enable32kHzPin(false);
  // Set SQW pin as alarm 1 interrupt pin
  ds3231.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmOne);
 
  // Alarm 1 configuration
  DS3231AlarmOne alarm1(
    0,
    0,
    0,
    0,
    DS3231AlarmOneControl_SecondsMatch);
 
  // Set alarm 1 with given configuration
  ds3231.SetAlarmOne(alarm1);
  
  // Called so that the alarm will trigger again
  ds3231.LatchAlarmsTriggeredFlags();
}

void Array_sort(float *array , int n)
{ 
    // declare some local variables
    int i=0 , j=0 , temp=0;

    for(i=0 ; i<n ; i++)
    {
        for(j=0 ; j<n-1 ; j++)
        {
            if(array[j]>array[j+1])
            {
                temp        = array[j];
                array[j]    = array[j+1];
                array[j+1]  = temp;
            }
        }
    }
}

// function to calculate the median of the array
float Find_avg(float array[] , int n)
{
    float sum=0.0;

    for (int i = 0; i < n; ++i)
    {
      sum += array[i];
    }

    float avg = sum / n;
    
    return avg;
}

void getSensorReadings(float *temp, float *hum, float *energy, float *wind, float *sunshine)
{
  float tempArr[9], humArr[9], wpmArr[9], windArr[9];
  
  for (int i = 0; i < 9; ++i) {
    // Get temperature and humidity readings from SHT31
    tempArr[i] = sht31.readTemperature();
    humArr[i] = sht31.readHumidity();
    
    // Get solar radiation estimation from lux (MAX44009)
    wpmArr[i] = max44009.GetWpm() * 1.415;
    
    // Get wind sensor reading from ADC (Gain = 2/3, LSB = 0.1875 mV)
    int16_t adcReading = ads1115.readADC_SingleEnded(WIND_SENSOR_ADDR);
    windArr[i] = (adcReading * 0.0001875) * 6;
    
    delay(50);
  }
  
  Array_sort(tempArr, 9);
  Array_sort(humArr, 9);
  Array_sort(wpmArr, 9);
  Array_sort(windArr, 9);
  
  *temp = Find_avg(tempArr, 9);
  *hum = Find_avg(humArr, 9);
  
  // Solar radiation assumed to be constant over 1 minute
  float wpm = Find_avg(wpmArr, 9);
  *energy = wpm * 60;
  
  if (wpm >= 120)
  {
    *sunshine = *sunshine + 1;
  }
  
  *wind = Find_avg(windArr, 9);
}

void accumReadings(float temp, float hum, float energy, float wind)
{
  if (readCount == 1) // First temperature reading in 1 hour
  {
    tempMax = temp;
    tempMin = temp;
  }
  else // Next temperature readings
  {
    if (temp > tempMax)
    {
      tempMax = temp;
    }
    else if (temp < tempMin)
    {
      tempMin = temp;
    }
  }
  if (readCount == 1) // First humidity reading in 1 hour
  {
    humMax = hum;
    humMin = hum;
  }
  else // Next humidity readings
  {
    if (hum > tempMax)
    {
      humMax = hum;
    }
    else if (hum < humMin)
    {
      humMin = hum;
    }
  }
  if (readCount == 1) // First wind reading in 1 hour
  {
    windMax = wind;
    windMin = wind;
  }
  else // Next wind readings
  {
    if (wind > windMax)
    {
      windMax = wind;
    }
    else if (wind < windMin)
    {
      windMin = wind;
    }
  }
  energyAccum += energy;
}

void hourlyAvg(float *tempMean, float *humMean, float *energy1Hour, 
               float *windMean, float *sunshineHour)
{
  *tempMean = (tempMax + tempMin) / 2;
  *humMean = (humMax + humMin) / 2;
  *energy1Hour = energyAccum;
  *windMean = (windMax + windMin) / 2;
  *sunshineHour = sunshineAccum / readCount;
}

void LoRaInit()
{
  // Initialize RFM95W module
  // 915 MHz, BW: 125 kHz, CR: 4/8, SF: 12, 17 dbm
  rf95.init();
  rf95.setFrequency(FREQ);
  rf95.setModemConfig(MODEM_CONFIG);
  rf95.setTxPower(TX_POWER);

  // Somehow this node address set to 255 no matter what
  // Need this for proper setting of this node address
  manager.setThisAddress(SENSOR_ADDR);
}

void composePayload(uint8_t *payload, RtcDateTime timestamp, float vBattery, 
                    float tempMean, float humMean, float energy1Hour, float windMean, 
                    float sunshineHour)
{
  // Payload =  Year(2) + Month(1) + Day(1) + Hour(1) + Minute(1) + Second(1) + 
  //            vBat(4) + temp(4) + hum(4) + energy(4) + wind(4) + sunshine (4)
  
  uint16_t year = timestamp.Year();
  payload[0] = (year >> 8)& 0xFF;
  payload[1] = year & 0xFF;
  payload[2] = timestamp.Month();
  payload[3] = timestamp.Day();
  payload[4] = timestamp.Hour();
  payload[5] = timestamp.Minute();
  payload[6] = timestamp.Second();
  
  uint8_t *vBatteryArray;
  vBatteryArray = (uint8_t *)(&vBattery);
  payload[7] = vBatteryArray[0];
  payload[8] = vBatteryArray[1];
  payload[9] = vBatteryArray[2];
  payload[10] = vBatteryArray[3];
  
  uint8_t *tempArray;
  tempArray = (uint8_t *)(&tempMean);
  payload[11] = tempArray[0];
  payload[12] = tempArray[1];
  payload[13] = tempArray[2];
  payload[14] = tempArray[3];
  
  uint8_t *humArray;
  humArray = (uint8_t *)(&humMean);
  payload[15] = humArray[0];
  payload[16] = humArray[1];
  payload[17] = humArray[2];
  payload[18] = humArray[3];
  
  uint8_t *energyArray;
  energyArray = (uint8_t *)(&energy1Hour);
  payload[19] = energyArray[0];
  payload[20] = energyArray[1];
  payload[21] = energyArray[2];
  payload[22] = energyArray[3];
  
  uint8_t *windArray;
  windArray = (uint8_t *)(&windMean);
  payload[23] = windArray[0];
  payload[24] = windArray[1];
  payload[25] = windArray[2];
  payload[26] = windArray[3];
  
  uint8_t *sunshineArray;
  sunshineArray = (uint8_t *)(&sunshineHour);
  payload[27] = sunshineArray[0];
  payload[28] = sunshineArray[1];
  payload[29] = sunshineArray[2];
  payload[30] = sunshineArray[3];
}

void sendReadings(uint8_t *payload, uint8_t len)
{
  // Blocking; timeout: 200 ms; retries: 3 
  if (manager.sendtoWait(payload, len, GATEWAY_ADDR))
  {
    Serial.println("Message successfully sent to gateway.");
  }
  else
  {
    Serial.println("Failed sending message.");
  }
}
