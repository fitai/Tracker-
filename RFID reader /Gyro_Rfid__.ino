#include <WiFi101.h>                         //for WiFi  
#include <PubSubClient.h>                    //for MQTT
#include <SPI.h>                             //not sure
#include <String.h>                          //our packages are srings
#include <math.h>                            //is this necessary?
#include "MPU9250.h"                         //for ralking with IMU       
#include "quaternionFilters.h"               //for yaw, pitch and roll
#include <wiring_private.h>
MPU9250 myIMU;                               //IMU class from MPU9250.h
                                             //packeges are being sent every (BUFFER_SIZE/FREQUENCY_HZ)= seconds
#define BUFFER_SIZE 15                       // container that sets the size of the packet, sample 15 times and send it (3 from all x y z)
#define FREQUENCY_HZ 50                      // samoling frequency there are some other delayes 
#define SCAN_TAG_DELAY 20  
#define RFID_RESET_PIN A4
#define wifi_led A1
#define mqtt_led A2
#define tag_led A3 

const char* ssid = "Ellipsis Jetpack A8A7";                 //your SSID
const char* password = "f6de37a5";          //your password

IPAddress ServeR = {52, 204, 229, 101};       // Amazon
WiFiClient WIFIclient;
PubSubClient client(WIFIclient);

//******fitai_variables******


char msg[1032];                                                                    // creating the message string 
char tag_msg[1032];
float root_mean_square;                                                            // variable for rms (??????)
float rms_acc;                                                                     // variable for acceleration (why rms????)
float dataBuffer_x[BUFFER_SIZE] =  {0};                                            // buffer variable in x,y, and z(temperoary memory hold location)
float dataBuffer_y[BUFFER_SIZE] =  {0};                                            // buffer 
float dataBuffer_z[BUFFER_SIZE] =  {0};
float dataBuffer_gx[BUFFER_SIZE] = {0};
float dataBuffer_gy[BUFFER_SIZE] = {0};
float dataBuffer_gz[BUFFER_SIZE] = {0};


unsigned long timeBuffer[BUFFER_SIZE] = {0};                                          // buffer for tim             
int buffer_position = 0;                                                              // buffer for position 
String packetTotal, packetHeader0,packetHeader1, packetHeader2, packetHeader3, a_x,a_y,a_z,g_x,g_y,g_z;
String packetData, packetData_1, packetTail, sampling_rate, time_stamp,rfid,packetTotal_1;
String collar_id = "555";
String tag_id = "ABCD123";
int pause = (1000 / FREQUENCY_HZ)-SCAN_TAG_DELAY;                                      // milisecon / over frequency for [time period]
float temp_x, temp_y, temp_z, temp_pitch, temp_roll, acceleration;
float gyro_x, gyro_y, gyro_z;
unsigned long millis_stamp;
const float pi = 3.14159267;
float time_out = 25;
boolean tagDetected;                                                                    // We can use this to continue program action if our
                                                                                        // reading seems like a real tag was detected.
char ourTag[10];                                                                        // We will use this to hold the interrogated tag's data.
String ourTag_st ;





void setup() {
  Wire.begin();                                                        
  pinMode(wifi_led, OUTPUT);                                          
  pinMode(mqtt_led, OUTPUT); 
  pinMode(tag_led, OUTPUT);
  pinMode(RFID_RESET_PIN, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);    //rx on 13
  tagDetected = true;
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    myIMU.MPU9250SelfTest(myIMU.SelfTest);                                    // Start by performing self test and repoas, myIMU.accelBias);
    myIMU.initMPU9250();
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    myIMU.initAK8963(myIMU.magCalibration);
  }
  else{
    while(1);  
    digitalWrite(mqtt_led, HIGH);                                   // Loop forever if communication doesn't happen   ??? + solve this 
    digitalWrite(wifi_led, HIGH);                                              
    }
  
  

  //connect_to_wifi();
  //connect_to_mqtt();
  
  packetHeader0 = "{\"header\": {\"collar_id\": ";                              // id change +++     -1 555 
  packetHeader1 = ",\"lift_id\": \"None\" ,\"sampling_rate\":";
  packetHeader2 = "},\"content\":{\"";
  rfid = "rfid";
  a_x = "a_x";
  a_y = "a_y";
  a_z = "a_z";
  g_x = "g_x";
  g_y = "g_y";
  g_z = "g_z";
  packetHeader3 = "\": [";
  sampling_rate = FREQUENCY_HZ;
  packetTail = "]}}";
}





void loop() {

  scanTag(); 
  String ourTag_st(ourTag);
  ourTag_st = "xxxxxx";
  
                                        
  /*if (WiFi.status() != WL_CONNECTED) {                                            //if not connected to wifi, reconnect
    digitalWrite(wifi_led, HIGH);                                                 //turn off the LED if its not on WiFi
    connect_to_wifi();                                                            // vvv funtion 
  }

  
  if (!client.connected()) {                                                       //if not connected to the MQTT server, reconnect
    digitalWrite(mqtt_led, HIGH);                                                  //turn off the LED if it's not connected on the MQTT
    reconnect();                                                                   // another function 
  }
  //read all the axis
  */
  
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);                                          // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

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
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] -
               myIMU.magbias[2];
  }//if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  myIMU.updateTime();
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  myIMU.delt_t = millis() - myIMU.count;
  if (myIMU.delt_t > pause) {
    recordData( ourTag_st);     //<<<<<<<<<<   Record ++ break record and transmit into two different branch to save + transmit data
     
  }
}


void connect_to_wifi() {
  Serial.print("Trying to connect to: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int count =0;
  
  while (WiFi.status() != WL_CONNECTED && count <= time_out  ) {
    delay(300);
    Serial.print(".");
    digitalWrite(wifi_led, HIGH);
    count++;
  }

  if(WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    delay(4000);
  }

   if(WiFi.status() != WL_CONNECTED) {
    Serial.println("Issue cannot be fixed:: Hardware attention needed !!!!");
    double_blink(200);
  }

  Serial.println("Connected");
  digitalWrite(wifi_led, LOW);
}


void connect_to_mqtt() {
  client.setServer(ServeR, 1883);               //1883 is standard MQTT port:: server ip: {52, 204, 229, 101}@ Amazon
  client.setCallback(callback);
  if (!client.connected()) {
    reconnect();
  }
}


void callback(char* topic, byte* payload, unsigned int length) {

  // handle message arrived
  //there are not gioing to be any messages received
  //for right now, so we can skip this
}


void reconnect() {
  int count =0;
  
  while (!client.connected()) 
  {
    
    if (client.connect("MKRclient")) 
    {
      
      client.publish("fitai", "Hi, I am a sensor");
      client.subscribe("fitai");
      client.publish("rfid","Hi, here is the tag name");
      digitalWrite(mqtt_led, LOW);
    }
    else {
      

      //blink LED while waiting to connect
      digitalWrite(mqtt_led, LOW);
      delay(300);
      digitalWrite(mqtt_led, HIGH);
      delay(300);
      digitalWrite(mqtt_led, LOW);
      delay(300);
      digitalWrite(mqtt_led, HIGH);
      delay(300);
      count++;
    }
  }
  
  if(count > time_out){
    double_blink(100);
    }
 }
    
void publish_tag(){
  
   // packetTotal2 = packetHeader0 + collar_id + ",\"content\":{\""+ rfid + packetHeader3 + ourTag + "]}}}" ;
    client.loop();
  //  packetTotal2.toCharArray(tag_msg, 100);
    client.publish("rfid",tag_msg);
    Serial.println(tag_msg);
  }


void double_blink(int tmx){
  while(1){
      digitalWrite(mqtt_led, HIGH);
      digitalWrite(wifi_led, HIGH);
      delay(tmx);
      digitalWrite(mqtt_led, LOW);
      digitalWrite(wifi_led, LOW);
      delay(tmx);
  }}


void tripple_blink(int tmx){
  while(1){
      digitalWrite(mqtt_led, HIGH);
      digitalWrite(wifi_led, HIGH);
      digitalWrite(tag_led, HIGH);
      delay(tmx);
      digitalWrite(mqtt_led, LOW);
      digitalWrite(wifi_led, LOW);
      digitalWrite(tag_led, LOW);
      delay(tmx);
  }
  
  }


void recordData(String ourTag_st) {

  
  myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *                               //calculating yaw, pithc and roll
                              *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                      - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
                      
                      
  myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                              *(getQ() + 2)));
                              
                              
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
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


  
  millis_stamp = millis();                        // stamp    vvvv
 

  if (temp_x < 1) {                               //get rid of the small noise
    dataBuffer_x[buffer_position] = 0;
  }                                               //get rid of the small noise
  
  else {
    dataBuffer_x[buffer_position] = temp_x;
  }

  
  if (temp_y < 1) {                               //get rid of the small noise
    dataBuffer_y[buffer_position] = 0;
  }                                               //get rid of the small noise
  else {
    dataBuffer_y[buffer_position] = temp_y;
  }


  
  if (temp_z < 1) {                               //get rid of the small noise
    dataBuffer_z[buffer_position] = 0;
  }                                               //get rid of the small noise
  else {
    dataBuffer_z[buffer_position] = temp_z;
  }

dataBuffer_gx[buffer_position] = gyro_x;
dataBuffer_gy[buffer_position] = gyro_y;
dataBuffer_gz[buffer_position] = gyro_z;
  
  timeBuffer[buffer_position] = millis_stamp;    //^^^^^






  
  
  if (buffer_position == BUFFER_SIZE - 1)                                          //if buffer is full, send the data
  {
    
     packetData = dataBuffer_x[0];
     
     for (int i = 1; i < BUFFER_SIZE; i++)
     {                                                                              //composing X packet           
          packetData = packetData + ',' + dataBuffer_x[i];  
       }

          //put all the parts of the packet together
   
     packetTotal = packetHeader0+ collar_id + packetHeader1 + sampling_rate + packetHeader2 + a_x + packetHeader3 + packetData + "],";  
  client.loop();                                                                 // update the loop, new publish

     packetData = dataBuffer_y[0];
     
     for (int i = 0; i < BUFFER_SIZE; i++)
     {   
         packetData = packetData +','+ dataBuffer_y[i];                             //composing Y packet             
       }
    
    packetTotal += "\"a_y\":[" + packetData + "]," ;                           //put all the parts of the packet together     ++++++++
  client.loop();                                                               // update the loop, new publish

    packetData = dataBuffer_z[0];
    
    for (int i = 0; i < BUFFER_SIZE; i++) 
    {                                      //composing Z packet          
       packetData = packetData + ',' + dataBuffer_z[i];  
     }
   
    packetTotal += "\"a_z\":[" + packetData + "],";                                   //put all the parts of the packet together     
  client.loop(); 


   
    packetData = dataBuffer_gx[0]; 
    
    for (int i = 0; i < BUFFER_SIZE; i++) 
    {                                           //composing G::X packet           
       packetData = packetData + ',' + dataBuffer_gx[i];  
     }
     
    packetTotal += "\"g_x\":[" + packetData + "]," ;                              //put all the parts of the packet together  
  client.loop(); 

    packetData = dataBuffer_gy[0];
    for (int i = 0; i < BUFFER_SIZE; i++) 
    {                                        //composing G::Y packet          
       packetData = packetData + ',' + dataBuffer_gy[i];  
     }
     
    packetTotal += "\"g_y\":[" + packetData + "]," ;                              //put all the parts of the packet together     
  client.loop(); 
  
    packetData = dataBuffer_gz[0];
    for (int i = 0; i < BUFFER_SIZE; i++) 
    {                                        //composing G::Z packet           
       packetData = packetData + ',' + dataBuffer_gz[i];  
      }
   
    packetTotal += "\"g_z\":[" + packetData + "],";                                 //put all the parts of the packet together      
  client.loop(); 

  
    
      
  /*if (buffer_position == BUFFER_SIZE - 1)                                          //if buffer is full, send the data
  {
    
     packetData_1 = ourTag_st;
     
     for (int i = 1; i < BUFFER_SIZE; i++)
     {                                                                              //composing X packet           
          packetData_1 + =  ',' + packetData_1;  
       }
  }
  */
  packetData_1 = ourTag_st;
       packetTotal_1 = packetHeader0 + collar_id + ",\"content\":{\""+ rfid + packetHeader3 + packetData_1 + "]}}}" ; /// working with substitute ourTag
    client.loop();
    
   
   
                                                                                

    //add time stamp**********************  
    
    packetData = timeBuffer[0];
    for (int i = 0; i < BUFFER_SIZE; i++) 
    {
       packetData = packetData + ',' + timeBuffer[i];
    
      }
    
    packetTotal += "\"millis\":[" + packetData + packetTail;
    packetTotal.toCharArray(msg, 1032);                                              //convert from string to char array
    packetTotal_1.toCharArray(tag_msg, 1032);
    
    //client.publish("fitai", msg );  
    //client.publish("rfid",tag_msg);
    
    //send the data

    
    Serial.println(msg);
    Serial.println(tag_msg);
    packetData_1 = "";
    packetData = "";                                                                 //clean the package
    buffer_position = 0;
  }

  else {
    buffer_position++;   }                                         //if buffer is not full, next piece of data goes to the next position
 

  myIMU.count = millis();
  myIMU.sumCount = 0;
  myIMU.sum = 0;
}                 




void scanTag(){

  // This function call will return the interrogated tag's data in form of a char
  // array. If there was an error in the integrity of the transmission, it will
  // return "0000000000". Of course, this only happens unless there is something
  // in the serial buffer. All the while, we also
  // set a flag if there was data there. This comes into use later on.
  
  if (Serial1.available() > 0)
  {
    delay(20);                  // Give some time for all data to arrive safe and sound into the buffer.



    // The if statement below ensures that the beginning of a tag is seen.
    // Remember that a Start of Text is the decimal value of 2. If we do not
    // see this, all bets are off in even continuing to look further into the
    // buffer. I use peek simply because I don't like touching data until I
    // decide to process it.
    
    if (Serial1.peek() != 2)
    {
      flushSerial1Buffer();     // Flush the buffer to bring it back to an initial, known state.
    }  
    else
    {

      // Go and process the tag in the serial1 buffer.
      // NOTE: The fetchTagData function actually alters the ourTag array
      // declared earlier before. Nothing is returned because if an array's name
      // pass along into a function, it is actually passing by reference, not by
      // value. That means we are changing the array's contents in the function
      // so nothing needs to be returned! There are other spots in this program
      // where this happens so please keep this in mind.
      
      fetchTagData(ourTag);   
      Serial.print("Your tag says it is: ");
      //Serial.flush();
      printTag(ourTag);
  }
 
  if (tagDetected)
  {
      //Serial.flush();
      }
  
  }
  
  else {
    Serial.println("something is wrong with the tag");
    }
}


void flushSerial1Buffer() 
{  

  // Now there is a function on the Arduino that is called Serial1.flush
  // but it does not really flush the incoming buffer in recent versions, please
  // check arduino.cc for more information on why. So instead, we will just
  // keep on plucking data off of the serial buffer until it is empty!
  
  while (Serial1.available() > 0)
  {
    Serial.read();
  }
}

void fetchTagData(char tempTag[]) 
{  
 // First, pluck off the Start of Text character
  Serial1.read();
   // Second, read off the tag's actual ID data
  for (int counter = 0; counter < 10; counter++)
  {
    tempTag[counter] = Serial1.read();
  }
 // Third, pluck off two checksum, one CR, and one LF characters
  Serial1.read();
  Serial1.read();
  Serial1.read();
  Serial1.read();
  // Fourth, pluck off what should be the End of Text character. And
  // while we are plucking, why not throw in a sanity check (mentioned in
  // the article)
  if (Serial1.read() != 3)
  {
    
    for (int counter = 0; counter < 10; counter++)
    {
      tempTag[counter] = '0';
    } 
  }  
  else
  {
    // But if it all looks good, flush the buffer and keep the previously
    // acquired data
    flushSerial1Buffer();
  }
}


void printTag(char tag[])
{
  // This function just helps identify what the tag ID is so that you
  // may initially read this in, hard code into your program, compile,
  // and then run to have a valid tag in your database.
  
  for (int counter = 0; counter < 10; counter++)
  {
    Serial.print(tag[counter]);
  }
  Serial.println("");
}




/*
void scanTag() {
  char tagString[13]; //There are 12 chars and a new line \n
  int index = 0;
  boolean reading = false;

  while (Serial1.available()) {

    int readByte = Serial1.read(); 
    if (readByte == 2) reading = true; //begining of tag
    if (readByte == 3) reading = false; //end of tag

    if (reading && readByte != 2 && readByte != 10 && readByte != 13) {
      tagString[index] = readByte;
      index ++;
    }
  }
  
  if(!Serial1){
    tripple_blink(200);
    }

  checkTag(tagString); //+print to serial+ assign to collar id 
  clearTag(tagString); 
  resetReader(); 
}


void checkTag(char tag[]) {
  if (strlen(tag) < 5) return; //empty, no need to contunue
  else {
    Serial.println(tag); //read out any unknown tag
    collar_id = tag;
  }
}


void clearTag(char one[]) {
  for (int i = 0; i < strlen(one); i++) {
    one[i] = 0;
  }
}


void resetReader() {
    digitalWrite(tag_led, LOW);
    digitalWrite(tag_led, HIGH);
    delay(20);
}
*/



