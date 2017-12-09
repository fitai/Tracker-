
#include <WiFi101.h>                          //for WiFi 

//***********************************************************************************************************************************
//***********************************************************************************************************************************
//Fit.Ai Tracker for Expo (08/09/2017)
//******ID+IP+WIFI.CLIENT****** VV edit here 
//#define mag_bias_null
String tracker_id = "555";
const char *mqtt_id = "MKRclient1";


const char* ssid = "Ellipsis Jetpack A8A7";   //your SSID
const char* password = "f6de37a5";            //your password

IPAddress ServeR = {52, 15, 200, 179};        // dev server
//IPAddress ServeR = {52, 204, 229, 101};       // Amazon aws

//***********************************************************************************************************************************
//***********************************************************************************************************************************
//***********************************************************************************************************************************

 
#include <PubSubClient.h>                     //for MQTT
#include <SPI.h>                              //spi com
#include <String.h>                           //our packages are srings
#include "MPU9250.h"                          //for ralking with IMU       
#include "quaternionFilters.h"                //for yaw, pitch and roll
#include <wiring_private.h>                   //added to run 
#include <math.h>                             //needed for drift calculation 

                       
#define  BUFFER_SIZE 15                       //container that sets the size of the packet, sample 15 times and send it (6 from all x y z, acc + gyro)
#define  FREQUENCY_HZ 30                      //sampling frequency 

MPU9250 myIMU; 


WiFiClient WIFIclient;                        //creating client for wifi
PubSubClient client(WIFIclient);              //client for mqtt


//******************************************************fitai_variables*****************************************************************                 
//Tripple blink for hardware faliure 
//Double blink for connection faliur
//yellow blink with 300ms gap == mqtt data connection problem



char msg[1032];                                                                    // creating the message string packet (1) 

float root_mean_square;                                                            // variable for rms (??????)
float rms_acc;                                                                     // variable for acceleration (why rms????)
float dataBuffer_x[BUFFER_SIZE] =  {0};                                            // buffer variable in x,y, and z(temperoary memory hold location)
float dataBuffer_y[BUFFER_SIZE] =  {0};                                            // buffer x, y, z, gz, gy and gz. 
float dataBuffer_z[BUFFER_SIZE] =  {0};
float dataBuffer_gx[BUFFER_SIZE] = {0};
float dataBuffer_gy[BUFFER_SIZE] = {0};
float dataBuffer_gz[BUFFER_SIZE] = {0};


unsigned long timeBuffer[BUFFER_SIZE] = {0};                                        // buffer for tim             
int    buffer_position = 0;                                                         // buffer for position 

String packetTotal, packetHeader0,packetHeader1, packetHeader2, packetHeader3;
String packetData, packetData_1, packetTail, sampling_rate, time_stamp,rfid,packetTotal_1;
String a_x,a_y,a_z,g_x,g_y,g_z;
float  temp_x, temp_y, temp_z, temp_pitch, temp_roll, acceleration;
float  gyro_x, gyro_y, gyro_z;
unsigned long millis_stamp;
const float pi = 3.14159267;


//***************************************************************************************************************************
//***************************************************************************************************************************
//*******************CAN EDIT************************(make changes only to the segments below)

            
int    pause = (1000 / FREQUENCY_HZ);                                // milisecon / over frequency for [time period]

float time_out = 25;
                                                               
                                                                                    // reading seems like a real tag was detected.


//***************************************************************************************************************************
//***************************************************************************************************************************
//***************************************************************************************************************************



void setup() 

{
  
  Wire.begin();                                                        

  
  Serial.begin (9600);
  

        

  
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71)                                                                    // WHO_AM_I should always be 0x68
  {
    myIMU.MPU9250SelfTest(myIMU.SelfTest);                                          // Start by performing self test and repoas, myIMU.accelBias);
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);                        // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.initMPU9250();
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    myIMU.initAK8963(myIMU.magCalibration);                                          // Get magnetometer calibration from AK8963 ROM
  }
  else
  {
    Serial.println("Imu not detected <<< Problem !!!!");  
   // tripple_blink(200);                                                             // all blink fast         
    }
    

  connect_to_wifi();
  connect_to_mqtt();
  
  packetHeader0 =              "{\"header\": {\"tracker_id\": ";                     // id change 
  packetHeader1 = ",\"lift_id\": \"None\" ,\"sampling_rate\":";
  packetHeader2 =                          "},\"content\":{\"";
  rfid="rfid";
  a_x = "a_x";
  a_y = "a_y";
  a_z = "a_z";
  g_x = "g_x";
  g_y = "g_y";
  g_z = "g_z";
  packetHeader3 = "\": [";
  packetTail      = "]}}";
  sampling_rate = FREQUENCY_HZ;
}





void loop() 

{
  
                                        
  if (WiFi.status() != WL_CONNECTED) 
  {                                                                               //if not connected to wifi, reconnect                                          
    connect_to_wifi();                                                            // vvv funtion 
  }

  
  if (!client.connected())
  {                                                                                //if not connected to the MQTT server, reconnect
    reconnect();                                                                   // another function 
  }
  

  
  
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);                                          // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);                // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;



#ifdef mag_bias_null            // zeroing out mag values 

myIMU.mx = 0;
myIMU.my = 0;
myIMU.mz = 0;
  }

#else
    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] -
               myIMU.magbias[2];
  }//if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
#endif


  myIMU.updateTime();
  
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  myIMU.delt_t = millis() - myIMU.count;

  
  if (myIMU.delt_t > pause) 
  {
    recordData();     
     
  }
  
}






void recordData() 
{

  
  myIMU.yaw   =                  atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *                      //calculating yaw, pithc and roll
                      *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                      - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
                      
                      
  myIMU.pitch =                   -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                                                         *(getQ() + 2)));
                              
                              
  myIMU.roll  =                  atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                       *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                       - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
                      
                      
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;
  myIMU.yaw   -= 8.5;
  myIMU.roll  *= RAD_TO_DEG;

  
  temp_pitch = myIMU.pitch / 360 * 2 * pi;                                            //converting from degrees to radias
  temp_roll = myIMU.roll / 360 * 2 * pi;

  
  temp_x = myIMU.ax;                                                                  //calculating acceleration (and RMS)
  temp_y = myIMU.ay;
  temp_z = myIMU.az;
  
  //take out the gravity
  
  temp_x += sin(temp_pitch);
  temp_y -= sin(temp_roll) * cos(temp_pitch);
  temp_z -= cos(temp_pitch) * cos(temp_roll);
  //convert to m/s^2
  
  temp_x *= 9.81;
  temp_y *= 9.81;
  temp_z *= 9.81;

  gyro_x = myIMU.gx;
  gyro_y = myIMU.gy;
  gyro_z = myIMU.gz;


  
  millis_stamp = millis();                        // stamp for time tracking
 

  if (temp_x < 1) 
  {                                               //get rid of the small noise
    dataBuffer_x[buffer_position] = 0;
  }                                               //get rid of the small noise
  
  else 
  {
    dataBuffer_x[buffer_position] = temp_x;
  }

  
  if (temp_y < 1) 
  {                                               //get rid of the small noise
    dataBuffer_y[buffer_position] = 0;
  }                                               //get rid of the small noise
  else 
  {
    dataBuffer_y[buffer_position] = temp_y;
  }


  
  if (temp_z < 1) 
  {                                               //get rid of the small noise
    dataBuffer_z[buffer_position] = 0;
  }                                               //get rid of the small noise
  else
  {
    dataBuffer_z[buffer_position] = temp_z;
  }

dataBuffer_gx[buffer_position] = gyro_x;
dataBuffer_gy[buffer_position] = gyro_y;
dataBuffer_gz[buffer_position] = gyro_z;

timeBuffer[buffer_position] = millis_stamp;    


  
  if (buffer_position == BUFFER_SIZE - 1)                                           //if buffer is full, send the data
  {
    
     packetData = dataBuffer_x[0];
     
     for (int i = 1; i < BUFFER_SIZE; i++)
     {                                                                              //composing X packet           
          packetData = packetData + ',' + dataBuffer_x[i];  
       }

          //put all the parts of the packet together
   
     packetTotal = packetHeader0+ tracker_id + packetHeader1 + sampling_rate + packetHeader2 + a_x + packetHeader3 + packetData + "],";  
  client.loop();                                                                    // update the loop, new publish



     packetData = dataBuffer_y[0];
     
     for (int i = 1; i < BUFFER_SIZE; i++)
     {   
         packetData = packetData +','+ dataBuffer_y[i];                             //composing Y packet             
       }
    
    packetTotal += "\"a_y\":[" + packetData + "]," ;                              //put all the parts of the packet together     ++++++++
  client.loop();                                                                    // update the loop, new publish



    packetData = dataBuffer_z[0];
    
    for (int i = 1; i < BUFFER_SIZE; i++) 
    {                                                                               //composing Z packet          
       packetData = packetData + ',' + dataBuffer_z[i];  
     }
   
    packetTotal += "\"a_z\":[" + packetData + "],";                                 //put all the parts of the packet together     
  client.loop(); 



    packetData = dataBuffer_gx[0]; 
    
    for (int i = 1; i < BUFFER_SIZE; i++) 
    {                                                                               //composing G::X packet           
       packetData = packetData + ',' + dataBuffer_gx[i];  
     }
     
    packetTotal += "\"g_x\":[" + packetData + "]," ;                              //put all the parts of the packet together  
  client.loop(); 



    packetData = dataBuffer_gy[0];
    
    for (int i = 1; i < BUFFER_SIZE; i++) 
    {                                                                               //composing G::Y packet          
       packetData = packetData + ',' + dataBuffer_gy[i];  
     }
     
    packetTotal += "\"g_y\":[" + packetData + "]," ;                              //put all the parts of the packet together     
  client.loop(); 


  
    packetData = dataBuffer_gz[0];
    
    for (int i = 1; i < BUFFER_SIZE; i++) 
    {                                                                               //composing G::Z packet           
       packetData = packetData + ',' + dataBuffer_gz[i];  
      }
   
    packetTotal += "\"g_z\":[" + packetData + "],";                                 //put all the parts of the packet together      
  client.loop(); 

  
    
  
    
    
    
    packetData = timeBuffer[0];
    
    for (int i = 1; i < BUFFER_SIZE; i++) 
    {
       packetData = packetData + ',' + timeBuffer[i];
    
      }
    
    packetTotal += "\"millis\":[" + packetData + packetTail;
    packetTotal.toCharArray(msg, 1032);                                             //convert from string to char array
    
    
    client.publish  ("fitai", msg);                                                 //send the data
    
    
  

    
    Serial.println(msg);
  
    
    packetData = "";                                                                //clean the package
    buffer_position = 0;
    
  }

  else 
  {
    buffer_position++;  
    }                                                                               //if buffer is not full, next piece of data goes to the next position
  

  myIMU.count = millis();
  myIMU.sumCount = 0;
  myIMU.sum = 0;
  
}         

        


void connect_to_wifi() 
{
  Serial.print("Trying to connect to: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int count =0;
  
  while (WiFi.status() != WL_CONNECTED && count <= time_out  )
  {
    delay(300);
    Serial.print(".WIFI !!..");
    count++;
  }

  if(WiFi.status() != WL_CONNECTED) 
  {
    WiFi.disconnect();
    Serial.println("Restarting WiFi module...!!");
    WiFi.begin(ssid, password);
    delay(4000);
  }

   if(WiFi.status() != WL_CONNECTED)
   {
    Serial.println("WIFI...Issue cannot be fixed:: Hardware attention needed !!!!");               
  }

  Serial.println("Connected to wifi");
}


void connect_to_mqtt() 
{
  client.setServer(ServeR, 1883);               //1883 is standard MQTT port:: server ip: {52, 204, 229, 101}@ Amazon
  client.setCallback(callback);
  
  if (!client.connected())
  {
    reconnect();
  }
}


void callback(char* topic, byte* payload, unsigned int length) 
{

  // handle message arrived
  //there are not gioing to be any messages received
  //for right now, so we can skip this
}


void reconnect() 
{
  int count =0;
  
  while (!client.connected()) 
  {
    
    if (client.connect(mqtt_id)) 
    {
      
      client.publish("fitai", "Hi, I am a sensor");
      client.subscribe("fitai");
      client.publish("rfid","Hi, here is the tag name");
      
    }
    
    else 
    {
      
      //blink LED while waiting to connect
     Serial.println("Mqtt connect problem...0x6C");
      count++;
      
   if(count > time_out)
  {
    Serial.println("mqtt time out....can not connect...0xT");
    }
   }
  }
 }
    

  




