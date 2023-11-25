//User defined configuration
#define STASSID "mywifinetwork"
#define STAPSK "abc123"

#define MQTTbrokerHOST "192.168.1.153"
#define MQTTport 1883
#define MQTTusername "mosquittouser"
#define MQTTpassword "xyz123"


//kopplingkort huzzah använder GPIO 4
//lös wemos d1+dallas använder GPIO 0
#define ONE_WIRE_BUS 4 //GPIO number for DS18B20 onewire bus
#define maxSensorCount 6 // Max connected DS18B20 sensors on the onewire bus

//tuning [here be dragons]
#define readingsPerCycle 50
#define restartInterval 1000*60*1.5
#define tickInterval 900
#define rollingAverageNumerOfValuesToIncludeOnEachSide 6
#define initialSelfheatDuringCycle 3 //bits

#define singleReadingCountWhenCalibrating 10
double ExpectedRampDownWhenCalibrating[] = {1 ,	0.5 ,	0.25 ,	0.15 ,	0.05 , 0 , 0 , 0 , 0 , 0};
#define requiredConstantReadingsForCalibration 5
unsigned long TuningRetryInterval = 166;




#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
#endif


// internal variables
#define lastBitWorth 0.0625
double rawReading[maxSensorCount][readingsPerCycle];
double selfHeatModel[maxSensorCount][readingsPerCycle];
double selfHeatSmoothed[maxSensorCount][readingsPerCycle];
double positionWeight[maxSensorCount][readingsPerCycle];
double temperature[maxSensorCount];
int usableReadingsCount[maxSensorCount];
int readingsActuallyUsedCount[maxSensorCount];
double singleReadingSeries[maxSensorCount][singleReadingCountWhenCalibrating];

double firstReadingStableTemperature[maxSensorCount];
int firstReadingStableTime[maxSensorCount];
double tuningCompensation[maxSensorCount];
unsigned int tuningCompensationUsed[maxSensorCount];


unsigned long THERMOMETER_uptime = 0;
unsigned long CPU_uptime = 0;
unsigned long WIFI_uptime = 0;
unsigned long MQTTCONNECTION_uptime = 0;
unsigned int singleReadingMode = 0;
unsigned int phase = 0;
const char* ssid = STASSID;
const char* password = STAPSK;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

unsigned long previousRestartPhaseMillis = 0;
unsigned long RestartPhaseInterval = restartInterval;

unsigned long previousTickMillis = 0;
unsigned long TickInterval = tickInterval;
int calibrationRecoveryIndex = 0;

unsigned int numberOfSensors = 0;
String deviceAddressPrinterFriendly[10];

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


void setupDallasSensors()
{
  sensors.begin();
  sensors.setResolution(12);
  numberOfSensors = sensors.getDeviceCount();
  for (int i = 0;  i < numberOfSensors;  i++)
  {
    DeviceAddress CurrentThermometerAddress; 
    bool foo = sensors.getAddress(CurrentThermometerAddress,i);

    String addrString = "";
    for (int j = 0; j < 8; j++) {
      if (CurrentThermometerAddress[j] < 16) {
        addrString += "0";
      }
      addrString += String(CurrentThermometerAddress[j], HEX) ;
    }
    deviceAddressPrinterFriendly[i] = addrString;
  }
}
void preFillModel()
{
for (int sensorIndex = 0;  sensorIndex < numberOfSensors;  sensorIndex++)
  {
    tuningCompensation[sensorIndex] = -0.15;
    for (int i = 0;  i < readingsPerCycle;  i++)
    {
      selfHeatModel[sensorIndex][i] = ((double) initialSelfheatDuringCycle*(double) lastBitWorth)*((double)i/(double)readingsPerCycle);
    }
  }
}

void setup() 
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Booting");
  setupDallasSensors();
  preFillModel();
  phase = 1000;
}

void setupWifiAndOta()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.waitForConnectResult();


  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  //No authentication by default
  ArduinoOTA.setPassword("flash");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


String filterString(String input) {
  String filteredString = "";

  for (int i = 0; i < input.length(); i++) {
    char currentChar = input.charAt(i);

    if (isHexCharacter(currentChar)) {
      filteredString += currentChar;
    }
  }

  return filteredString;
}

bool isHexCharacter(char c) {
  if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
    return true;
  }

  return false;
}


void mqttAutoDiscoveryPublishConfig() {
  String uniqueSensorIdString;
  String configPayload;
  String filteredMacAddr = filterString(WiFi.macAddress());
  String deviceInfoString = "\"device\":{\"ids\":\""+ filteredMacAddr + "\",\"sw\":\"" + (String)__DATE__ + (String)" " + (String)__TIME__ "\",\"name\":\"Multitempsensor\"},";

  for (int sensorNumber = 0; sensorNumber < numberOfSensors; sensorNumber++) 
  {
    
    // Define the sensor configuration payload
    String filteredSensorAddr = filterString(deviceAddressPrinterFriendly[sensorNumber]);
    uniqueSensorIdString = String("ds") + filteredSensorAddr;

    configPayload = "{"; 
    //  configPayload += "\"name\": \"" + uniqueSensorIdString + "\",";
    configPayload += "\"stat_t\":\"homeassistant/sensor/" + uniqueSensorIdString + "/temp\",";
    configPayload += "\"unit_of_meas\":\"°C\",";
    configPayload += "\"exp_aft\":\"300\",";
    configPayload += "\"frc_upd\":\"true\",";
    configPayload += deviceInfoString;
    configPayload += "\"uniq_id\":\"" + uniqueSensorIdString + "\"}";
    // Publish the configuration payload to the MQTT discovery topic
    mqttClient.beginMessage("homeassistant/sensor/" + uniqueSensorIdString + "/config");
    mqttClient.print(String((const char*)configPayload.c_str()));
    mqttClient.endMessage();

  }



}



void connectMQTT()
{
  mqttClient.setUsernamePassword(MQTTusername, MQTTpassword);
  mqttClient.connect(MQTTbrokerHOST, MQTTport);
}

void connectionMonitor()
{
  if (WiFi.status() == WL_CONNECTED)
  {
//    Serial.println("wifi is connected! Wifi uptime:" + String(WIFI_uptime));
    if (!mqttClient.connected()) 
    {
      Serial.println("mqttClient.connected = false! Attempting connect...");
      MQTTCONNECTION_uptime = 0;
      connectMQTT();
    }
    else
    {
      mqttClient.poll();
//      Serial.println("mqttClient.connected = true! MQTTCONNECTION_uptime: " + String(MQTTCONNECTION_uptime));
    }

    ArduinoOTA.handle();    
  }
  else 
  {
    Serial.println("wifi is not connected!");
    WIFI_uptime = 0;
    setupWifiAndOta();
  }
}

void systemMQTTreport()
{
  mqttClient.beginMessage("multitempsensor/arduinoboard" + WiFi.macAddress() + "/phase");
  mqttClient.print(String(phase));
  mqttClient.endMessage();

  mqttClient.beginMessage("multitempsensor/arduinoboard" + WiFi.macAddress() + "/CPU_uptime");
  mqttClient.print(String(CPU_uptime));
  mqttClient.endMessage();

  mqttClient.beginMessage("multitempsensor/arduinoboard" + WiFi.macAddress() + "/WIFI_uptime");
  mqttClient.print(String(WIFI_uptime));
  mqttClient.endMessage();

  mqttClient.beginMessage("multitempsensor/arduinoboard" + WiFi.macAddress() + "/MQTTCONNECTION_uptime");
  mqttClient.print(String(MQTTCONNECTION_uptime));
  mqttClient.endMessage();

  mqttClient.beginMessage("multitempsensor/arduinoboard" + WiFi.macAddress() + "/numberOfSensors");
  mqttClient.print(String(numberOfSensors));
  mqttClient.endMessage();

  mqttClient.beginMessage("multitempsensor/arduinoboard" + WiFi.macAddress() + "/readingsPerCycle");
  mqttClient.print(String(readingsPerCycle));
  mqttClient.endMessage();

  mqttAutoDiscoveryPublishConfig();
  
/*
  //ad hoc debug output as needed
  for (int i = 0; i < numberOfSensors; i++) 
  {
    String arrayString = "";
    for (int j = 0; j < readingsPerCycle; j++) {
      arrayString += String(positionWeight[i][j], 5);
      if (j < readingsPerCycle - 1) {
        arrayString += ";";
      }
    }
    mqttClient.beginMessage("multitempsensor/debugprint_positionWeight_sensor" + String(i));
    mqttClient.print(arrayString);
    mqttClient.endMessage();
  }
    for (int i = 0; i < numberOfSensors; i++) {
    String arrayString = "";
    for (int j = 0; j < readingsPerCycle; j++) {
      arrayString += String(selfHeatModel[i][j], 4);
      if (j < readingsPerCycle - 1) {
        arrayString += ";";
      }
    }
    mqttClient.beginMessage("multitempsensor/debugprint_selfHeatModel_sensor" + String(i));
    mqttClient.print(arrayString);
    mqttClient.endMessage();
  }
*/


/*
  //ad hoc debug output as needed
  for (int i = 0; i < numberOfSensors; i++) {
    String arrayString = "";
    for (int j = 0; j < readingsPerCycle; j++) {
      arrayString += String(selfHeatModel[i][j], 4);
      if (j < readingsPerCycle - 1) {
        arrayString += ";";
      }
    }
    mqttClient.beginMessage("multitempsensor/debugprint_selfHeatModel_sensor" + String(i));
    mqttClient.print(arrayString);
    mqttClient.endMessage();
  }

  //ad hoc debug output as needed
  for (int i = 0; i < numberOfSensors; i++) {
    String arrayString = "";
    for (int j = 0; j < readingsPerCycle; j++) {
      arrayString += String(selfHeatSmoothed[i][j], 4);
      if (j < readingsPerCycle - 1) {
        arrayString += ";";
      }
    }
    mqttClient.beginMessage("multitempsensor/debugprint_selfHeatSmoothed_sensor" + String(i));
    mqttClient.print(arrayString);
    mqttClient.endMessage();
  }

*/
  //ad hoc debug output as needed
  for (int i = 0; i < numberOfSensors; i++) 
  {
    String arrayString = "";
    for (int j = 0; j < singleReadingCountWhenCalibrating; j++) {
      arrayString += String(singleReadingSeries[i][j], 5);
      if (j < readingsPerCycle - 1) {
        arrayString += ";";
      }
    }
    mqttClient.beginMessage("multitempsensor/debugprint_singleReadingSeries_sensor" + String(i));
    mqttClient.print(arrayString);
    mqttClient.endMessage();
  }
/*
  for (int i = 0; i < numberOfSensors; i++) 
  {
    mqttClient.beginMessage("multitempsensor/debugprint_temperature_sensor" + String(i));
    mqttClient.print(String(temperature[i], 4));
    mqttClient.endMessage();
  }

  for (int i = 0; i < numberOfSensors; i++) 
  {
    mqttClient.beginMessage("multitempsensor/debugprint_readingsUsed_sensor" + String(i));
    mqttClient.print(String(readingsUsed[i]));
    mqttClient.endMessage();
  }  
*/

}


void PeriodicPhaseResetter()
{
  //Restart phase to zero every x minutes
  unsigned long currentRestartPhaseMillis = millis();
  if (currentRestartPhaseMillis - previousRestartPhaseMillis >= RestartPhaseInterval) 
  {
    previousRestartPhaseMillis = currentRestartPhaseMillis;
    phase = 0;
    WIFI_uptime++;
    MQTTCONNECTION_uptime++;
    CPU_uptime++;
    if (tuningCompensationCompleted() == false && (CPU_uptime-6) % TuningRetryInterval == 0)
    {
      singleReadingMode = singleReadingCountWhenCalibrating;
    }
  }
}

void readRawTemperatures()
{ 
  for (int i = 0;  i < numberOfSensors;  i++)
  {
      double tempC = sensors.getTempCByIndex(i);
      rawReading[i][phase-1] = tempC;
      mqttClient.beginMessage("multitempsensor/dallas_" + deviceAddressPrinterFriendly[i] + "/latestRawTemperatureReading");
      mqttClient.print(String(tempC,3));
      mqttClient.endMessage();
  }
}

void reportResults()
{
  for (int i = 0;  i < numberOfSensors;  i++)
  {
    String filteredSensorAddr = filterString(deviceAddressPrinterFriendly[i]);
    String uniqueSensorIdString = String("ds") + filteredSensorAddr;

    mqttClient.beginMessage("homeassistant/sensor/" + uniqueSensorIdString + "/temp");
    mqttClient.print(String(temperature[i],4));
    mqttClient.endMessage();
  }
}
void DoCalculationsSingleReading()
{
  for (int sensorIndex = 0;  sensorIndex < numberOfSensors;  sensorIndex++)
  {
    int singleReadingIndex = singleReadingCountWhenCalibrating-singleReadingMode;
    temperature[sensorIndex] = rawReading[sensorIndex][0] + ExpectedRampDownWhenCalibrating[singleReadingIndex]*tuningCompensation[sensorIndex];
    singleReadingSeries[sensorIndex][singleReadingIndex] = rawReading[sensorIndex][0];
  }
  calibrationRecoveryIndex = 0;
}
void DoCalculationsNormal()
{


  double selfHeatModelAvgFactor = 0.01;
  if (THERMOMETER_uptime > 1000)   selfHeatModelAvgFactor = 0.001;

  for (int sensorIndex = 0;  sensorIndex < numberOfSensors;  sensorIndex++)
  {

    //Determine stable temperature or not
    if (rawReading[sensorIndex][0] == firstReadingStableTemperature[sensorIndex])
    {
      firstReadingStableTime[sensorIndex]++;
    } else
    {
      firstReadingStableTemperature[sensorIndex]  = rawReading[sensorIndex][0];
      firstReadingStableTime[sensorIndex] = 0;
    }


    for (int readingIndex = 0;  readingIndex < readingsPerCycle;  readingIndex++)
    {
      double thisValue = rawReading[sensorIndex][readingIndex] - rawReading[sensorIndex][0];
      selfHeatModel[sensorIndex][readingIndex] = selfHeatModel[sensorIndex][readingIndex] * (1-selfHeatModelAvgFactor) + thisValue*selfHeatModelAvgFactor;
    }

    for (int readingIndex = rollingAverageNumerOfValuesToIncludeOnEachSide;  readingIndex < (readingsPerCycle - rollingAverageNumerOfValuesToIncludeOnEachSide);  readingIndex++)
    {
      double averagedValue = 0;
      for (int includeIndex = readingIndex-rollingAverageNumerOfValuesToIncludeOnEachSide; includeIndex < readingIndex+rollingAverageNumerOfValuesToIncludeOnEachSide+1 ; includeIndex++)
      {
        averagedValue += selfHeatModel[sensorIndex][includeIndex]/(1+2*rollingAverageNumerOfValuesToIncludeOnEachSide);
      }
      selfHeatSmoothed[sensorIndex][readingIndex] = averagedValue;
    }

    //Determine a suitable interval to use starting at the highestUsableIndex by stepping backwards as long as bit worth of readings (in the averaged model) or reaching lowestUsableIndex
    int highestUsableIndex = (readingsPerCycle - rollingAverageNumerOfValuesToIncludeOnEachSide)-1;
    int lowestUsableIndex = rollingAverageNumerOfValuesToIncludeOnEachSide+1;
    int lowestActuallyUsedIndex;

    bool moreReadings = true;
    int avgIndex = highestUsableIndex;

    while (moreReadings)
    {
      lowestActuallyUsedIndex = avgIndex;
      avgIndex--;
      if (selfHeatSmoothed[sensorIndex][(readingsPerCycle - rollingAverageNumerOfValuesToIncludeOnEachSide)-1] - selfHeatSmoothed[sensorIndex][avgIndex] > lastBitWorth) moreReadings = false;
      if (avgIndex < lowestUsableIndex) moreReadings = false; // out of range
    }

    usableReadingsCount[sensorIndex] = highestUsableIndex - lowestUsableIndex +1;
  
    //Calculate positionWeight
    for (int readingIndex = lowestUsableIndex;  readingIndex <= highestUsableIndex;  readingIndex++)
    {
      positionWeight[sensorIndex][readingIndex] = selfHeatSmoothed[sensorIndex][readingIndex] - selfHeatSmoothed[sensorIndex][readingIndex-1];
    }

    //Calculate total weight for used samples
    double totalWeight = 0;
    for (int readingIndex = lowestActuallyUsedIndex;  readingIndex <= highestUsableIndex;  readingIndex++)
    {
      totalWeight += positionWeight[sensorIndex][readingIndex];
    }

    //Calculate temperature  for used samples with weighting
    readingsActuallyUsedCount[sensorIndex] = 0;
    temperature[sensorIndex] = 0;
    for (int readingIndex = lowestActuallyUsedIndex;  readingIndex <= highestUsableIndex;  readingIndex++)
    {
      readingsActuallyUsedCount[sensorIndex]++;
      double relativeWeightingFactor = positionWeight[sensorIndex][readingIndex] / totalWeight;
      temperature[sensorIndex] += (rawReading[sensorIndex][readingIndex] - selfHeatSmoothed[sensorIndex][readingIndex])*relativeWeightingFactor;
    }
    int calibrationRecoveryIndexToUse = calibrationRecoveryIndex-0; 
    if (calibrationRecoveryIndexToUse < 0 ) calibrationRecoveryIndexToUse = 0;
    temperature[sensorIndex] += tuningCompensation[sensorIndex]*(1-ExpectedRampDownWhenCalibrating[calibrationRecoveryIndexToUse]);
    if (totalWeight == 0) temperature[sensorIndex] = rawReading[sensorIndex][0];
  }
  THERMOMETER_uptime++;
  calibrationRecoveryIndex++;
  if (calibrationRecoveryIndex > singleReadingCountWhenCalibrating-1) calibrationRecoveryIndex = singleReadingCountWhenCalibrating-1;
}

bool tuningCompensationCompleted()
{
  int compensatedSensorCount = 0;
  for (int i = 0; i < numberOfSensors; i++) 
  {
    if (tuningCompensationUsed[i] > 15) compensatedSensorCount++;
  }
  if (compensatedSensorCount == numberOfSensors) 
  {
    return true;
  }
  else
  {
    return false;
  }
}


void EvaluateTuningCompensation()
{
  for (int i = 0; i < numberOfSensors; i++) 
  {
    int j = singleReadingCountWhenCalibrating-1; 
    double endValue = singleReadingSeries[i][j];
    int numberOfIdenticalReadings = 0;
    while (j >= 0)
    {
      if (singleReadingSeries[i][j] == endValue && endValue > -666)
      {
        numberOfIdenticalReadings++;
      } else
      {
        endValue = -999;
      }
      Serial.println("sensor: " + String(i) + " singleReadingIndex: " + String(j) + " value: " + String(singleReadingSeries[i][j],4));
      j--;
    }
    Serial.println("sensor: " + String(i) + " numberOfIdenticalReadings in end: " + String(numberOfIdenticalReadings));
    if (numberOfIdenticalReadings >= requiredConstantReadingsForCalibration) 
    {
      double newValue = singleReadingSeries[i][singleReadingCountWhenCalibrating-1] - singleReadingSeries[i][0];
      if (tuningCompensationUsed[i] == 0)
      {
        tuningCompensation[i] = newValue;
      } else
      {
        tuningCompensation[i] = 0.75 * tuningCompensation[i] + 0.25 * newValue;
      }
      tuningCompensationUsed[i]++;
    }
  } 
}

void PerformTickActions()
{
  if (singleReadingMode == 0)
  {
    if (phase >= 1 && phase <=readingsPerCycle)  readRawTemperatures();
    if (phase < readingsPerCycle) sensors.requestTemperatures();
    if (phase == readingsPerCycle+1) 
    {
      DoCalculationsNormal();
      reportResults();
    }
  } else if (singleReadingMode > 0)
  {
    if (phase == 1) readRawTemperatures();
    if (phase == 0) sensors.requestTemperatures();
    if (phase == readingsPerCycle+1 ) //avoid jitter by delaying result
    {
      DoCalculationsSingleReading();
      reportResults();
      singleReadingMode--;
      if (singleReadingMode == 0) EvaluateTuningCompensation();
    }
  }

  systemMQTTreport();
  phase++;
}

void loop() 
{
  connectionMonitor();


  unsigned long currentTickMillis = millis();
  if (currentTickMillis - previousTickMillis >= TickInterval) 
  {
    previousTickMillis = currentTickMillis;
    PerformTickActions();
  }


  PeriodicPhaseResetter();
}
