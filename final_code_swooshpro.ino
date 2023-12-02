
#include <Wire.h>
#include <WiFi.h>
#include <MPU9250.h>
MPU9250 IMU(Wire, 0x68);

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//const int MPU_addr=0x68;  // I2C address of the MPU-9250
//int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// assign the ESP32 pins
#define D1 18 //SCK
#define D2 23 //SDI
#define D5 19  //SDO
#define D3 4  //CS1
#define D4 5  //CS2

// assign the SPI bus to pins
#define BME_SCK D1
#define BME_MISO D5  //SDO
#define BME_MOSI D2  //SDI
#define BME1_CS D3
#define BME2_CS D4

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme1(BME1_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
Adafruit_BME280 bme2(BME2_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;


// WiFi network info.
const char *ssid =  "realme1";     // Enter your WiFi Name
const char *pass =  "123456788"; // Enter your WiFi Password
WiFiServer server(80);


//MPU9250 IMU(Wire, 0x68); /undo this if doesnt work-Umama
//int status;

const int numSamples = 500;  // Adjust the number of samples as needed
float accelOffsets[3] = {0.0, 0.0, 0.0};
float gyroOffsets[3] = {0.0, 0.0, 0.0};
//float force[1] = {0.0};
float mass = 0.3;

void calibrateMPU9250() {
    Serial.println("Calibrating MPU9250. Keep the sensor still...");
    delay(2000);

    for (int i = 0; i < numSamples; ++i) {
        IMU.readSensor();
        accelOffsets[0] += IMU.getAccelX_mss();
        accelOffsets[1] += IMU.getAccelY_mss();
        accelOffsets[2] += IMU.getAccelZ_mss();
        gyroOffsets[0] += IMU.getGyroX_rads();
        gyroOffsets[1] += IMU.getGyroY_rads();
        gyroOffsets[2] += IMU.getGyroZ_rads();
        delay(10);
    }

    for (int i = 0; i < 3; ++i) {
        accelOffsets[i] /= numSamples;
        gyroOffsets[i] /= numSamples;
    }

    Serial.println("Calibration complete.");
    Serial.print("Accelerometer Offsets: ");
    Serial.print(accelOffsets[0]);
    Serial.print(", ");
    Serial.print(accelOffsets[1]);
    Serial.print(", ");
    Serial.println(accelOffsets[2]);
    Serial.print("Gyroscope Offsets: ");
    Serial.print(gyroOffsets[0]);
    Serial.print(", ");
    Serial.print(gyroOffsets[1]);
    Serial.print(", ");
    Serial.println(gyroOffsets[2]);
}

void setup() {
Serial.begin(9600);
Wire.begin();  
IMU.begin();
calibrateMPU9250();

//Wire.beginTransmission(IMU);
//Wire.write(0x6B);  // PWR_MGMT_1 register
 //Wire.write(0);     // set to zero (wakes up the MPU-9250)
 Wire.endTransmission(true);
 Serial.println("Wrote to pmu");
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");              // print ... till not connected
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address is : ");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("Server started");

  Serial.println(F("BME280 test"));

    bool status;
    
    // default settings
    status = bme1.begin();
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor 1 , check wiring!");
    }

    // default settings
    status = bme2.begin();
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor 2 , check wiring!");
    }    
    
    Serial.println("-- Default Test for BME280--");
    delayTime = 5000;

    Serial.println();

}


void loop(){
  printValues();
  delay(delayTime);


 // read the sensor
  IMU.readSensor();
  // display the data
  Serial.print("AccelX value: ");
  Serial.print(IMU.getAccelX_mss()-accelOffsets[0], 6);
  Serial.print("\t");
  Serial.print("AccelY value: ");
  Serial.print(IMU.getAccelY_mss()-accelOffsets[1], 6);
  Serial.print("\t");
  Serial.print("AccelZ value: ");
  Serial.print(IMU.getAccelZ_mss()-accelOffsets[2], 6);
  Serial.print("\t");
  Serial.print("Force Applied on racket: ");
  float force[1] = {0.0};
  force[1] += mass * sqrt(pow((IMU.getAccelX_mss()-accelOffsets[0]), 2)+ pow((IMU.getAccelY_mss()-accelOffsets[1]), 2)+pow((IMU.getAccelZ_mss()-accelOffsets[2]), 2));
  Serial.print(force[1], 6);
  Serial.print("\t");
  Serial.print("GyroX value: ");
  Serial.print(IMU.getGyroX_rads()-gyroOffsets[0], 6);
  Serial.print("\t");
  Serial.print("GyroY value: ");
  Serial.print(IMU.getGyroY_rads()-gyroOffsets[1], 6);
  Serial.print("\t");
  Serial.print("GyroZ value: ");
  Serial.println(IMU.getGyroZ_rads()-gyroOffsets[2], 6);
//  Serial.print("\t");
  /*Serial.print(IMU.getMagX_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(), 6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(), 6);*/      //deleting magnetometer and temperature from the readings
  delay(100);


WiFiClient client = server.available();
 if (client) 
  {                             
    Serial.println("new client");          
    String currentLine = "";                   //Storing the incoming data in the string
    while (client.connected()) 
    {            
      if (client.available())                  //if there is some client data available
      {                
        char c = client.read();                // read a byte
          if (c == '\n')                       // check for newline character, 
          {                     
          if (currentLine.length() == 0)      //if line is blank it means its the end of the client HTTP request
          {     
            client.print("<html><head><title> SwooshPro </title><meta http-equiv=\"refresh\" content=\"2\"></head></html>");
            client.print("<body bgcolor=\"#E6E6FA\"><h1 style=\"text-align: center; color: blue\">SwooshPro Accelerometer & Gyroscope Data On WEB SERVER</h1>");
            client.print("<p style=\"text-align: left; color: red; font-size:150% \">Accelerometer Values: ");
            client.print("<p style=\"text-align: left; font-size:150% \">AcX: ");
            client.print(IMU.getAccelX_mss()-accelOffsets[0], 6);
            client.print(" m/s^2");
            client.print("<br/>AcY: ");
            client.print(IMU.getAccelY_mss()-accelOffsets[1], 6);
            client.print(" m/s^2");
            client.print("<br/>AcZ: ");
            client.print(IMU.getAccelZ_mss()-accelOffsets[2], 6);
            client.print(" m/s^2");
            client.print("<p style=\"text-align: left; color: red; font-size:150% \">Ball Impact (Force Applied): ");
            client.print("<p style=\"text-align: left; font-size:150% \">Force: ");
            client.print(force[1], 6);
            client.print(" N");
            client.print("<p style=\"text-align: left; color: red; font-size:150% \">Gyroscope Values: ");
            client.print("<p style=\"text-align: left; font-size:150% \">Roll (GyX): ");
            client.print(IMU.getGyroX_rads()-gyroOffsets[0], 6);
            client.print(" rad/s");
            client.print("<br/>Pitch (GyY): ");
            client.print(IMU.getGyroY_rads()-gyroOffsets[1], 6);
            client.print(" rad/s");
            client.print("<br/>Yaw (GyZ): ");
            client.print(IMU.getGyroZ_rads()-gyroOffsets[2], 6);
            client.print(" rad/s");
            client.print("</p></body>");
            client.print("<p style=\"text-align: left; color: red; font-size:150% \">Height of racket: ");
            client.print("<p style=\"text-align: left; font-size:150% \">Height w.r.t. sensor1: ");
            client.print(bme1.readAltitude(SEALEVELPRESSURE_HPA)-291.14);
            client.print(" m");  
            client.print("<p style=\"text-align: left; font-size:150% \">Height w.r.t. sensor2: ");
            client.print(bme2.readAltitude(SEALEVELPRESSURE_HPA)-288.68);
            client.print(" m");   
            client.print("<p style=\"text-align: left; color: red; font-size:150% \">Pressure : ");
            client.print("<p style=\"text-align: left; font-size:150% \">Pressure using sensor1: ");
            client.print(bme1.readPressure() / 100.0F);
            client.print(" m");  
            client.print("<p style=\"text-align: left; font-size:150% \">Pressure using sensor1: ");
            client.print(bme2.readPressure() / 100.0F);
            client.print(" m");
            client.print("<p style=\"text-align: left; color: red; font-size:150% \">Temperature : ");
            client.print("<p style=\"text-align: left; font-size:150% \">Temp. value: ");
            client.print(bme1.readTemperature());
            client.print(" Degree Centigrade");      
            client.print("<p style=\"text-align: left; color: red; font-size:150% \">Humidity : ");
            client.print("<p style=\"text-align: left; font-size:150% \">Humidity value: ");
            client.print(bme1.readHumidity());  
            break;  // break out of the while loop:
          } 
           else
          {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
         } 
         else if (c != '\r') 
         {  // if you got anything else but a carriage return character,
          currentLine += c;       // add it to the end of the currentLine
         }
        }
      }
  }
}

void printValues() {

    Serial.print("Pressure 1= ");
    Serial.print(bme1.readPressure() / 100.0F);
    Serial.println(" hPa");
    Serial.print("Pressure 2= ");
    Serial.print(bme2.readPressure() / 100.0F);
    Serial.println(" hPa");    


    Serial.print("Approx. Altitude 1= ");
    Serial.print(bme1.readAltitude(SEALEVELPRESSURE_HPA)-291.14);
    Serial.println(" m");
    Serial.print("Approx. Altitude 2= ");
    Serial.print(bme2.readAltitude(SEALEVELPRESSURE_HPA)-288.68);
    Serial.println(" m");   

    Serial.print("Humidity = ");
    Serial.print(bme1.readHumidity());
    Serial.println(" %");

    Serial.print("Temperature = ");
    Serial.print(bme1.readTemperature());
    Serial.println(" *C");

    Serial.println();
}