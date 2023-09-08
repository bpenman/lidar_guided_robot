#include <Servo.h>
#include <Wire.h>
#include <Ultrasonic.h>
#include "SSD1306.h"
#include <ESP8266WiFi.h>  
#include "PubSubClient.h"
#include "WiFiManager.h" 

//definitions
#define motor_leftpin D0
#define motor_rightpin D2

//initialize objects
SSD1306 display(0x3c, D14, D15);
Ultrasonic ultrasonic_one(D8, D5);//right sensor
Ultrasonic ultrasonic_two(D10, D7);//left sensor
Ultrasonic ultrasonic_three(D9, D6); //front sensor
//Ultrasonic ultrasonic_four(D13, D12); //middle sensor

Servo motor_left;//left
Servo motor_right;//right

//variables
int distance_right;
int distance_left;
int distance3;
int distance_mid;

int motor_control_left = 90;
int motor_control_right = 90;

int throttle;//controls movement
int tar_arr_pos = 0;//position in target array

int x_1, y_1, x_2, y_2, xT, yT = -1;

//Setting up the target destination, xt[]={A,B,C,D ~} 
int xt[] = { 1400,650,350,150,1240,2000 };
int yt[] = { 150,150,400,50,660,700 };
//            t1  t2  t3  d4  t4 //tar_arr_pos = 4
////////////////////////////////////////////////////////////////////////////////////////////////

//MQTT Communication associated variables
char payload_global[100];
boolean flag_payload;

//MQTT Setting variables
const char* mqtt_server = "155.246.62.110";   //MQTT Broker(Server) Address
const char* MQusername = "jojo";              //MQTT username
const char* MQpassword = "hereboy";           //MQTT password
const char* MQtopic = "louis_lidar1";         //MQTT Topic for Arena_1 (EAS011 - South)
//const char* MQtopic = "louis_lidar2";       //MQTT Topic for Arena_2 (EAS011 - North)
const int mqtt_port = 1883;                   //MQTT port#

//Stevens WiFi Setting variables
const char* ssid = "Stevens-IoT";             //Stevens Wi-Fi SSID (Service Set IDentifier)   
const char* password = "nMN882cmg7";          //Stevens Wi-Fi Password

//WiFi Define
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
    delay(10);
    // We start by connecting to a Stevens WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    randomSeed(micros());
}

void callback(char* topic, byte* payload, unsigned int length) {
    for (int i = 0; i < length; i++) {
        payload_global[i] = (char)payload[i];
    }
    payload_global[length] = '\0';
    flag_payload = true;
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect                     
        if (client.connect(clientId.c_str(), MQusername, MQpassword)) {
            client.subscribe(MQtopic);              //EAS011 South - "louis_lidar1"       EAS011 North - "louis_lidar2"
        }
        else {
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////

//make displlaying readings take up way less lines
void display_readings() {

    //subscribe the data from MQTT server
    if (!client.connected()) {
        Serial.print("...");
        reconnect();
    }
    client.loop();

    String payload(payload_global);
    int testCollector[10];
    int count = 0;
    int prevIndex, delimIndex;

    prevIndex = payload.indexOf('[');
    while ((delimIndex = payload.indexOf(',', prevIndex + 1)) != -1) {
        testCollector[count++] = payload.substring(prevIndex + 1, delimIndex).toInt();
        prevIndex = delimIndex;
    }
    delimIndex = payload.indexOf(']');
    testCollector[count++] = payload.substring(prevIndex + 1, delimIndex).toInt();

    //Robot location x,y from MQTT subscription variable testCollector 
    x_2 = testCollector[0];
    y_2 = testCollector[1];

    char Xv[50]; //sets array for values
    char Yv[50];
    sprintf(Xv, "%d", x_2);
    sprintf(Yv, "%d", y_2);

    display.drawString(0, 0, "X-Axis:");
    display.drawString(60, 0, Xv);
    display.drawString(0, 16, "Y-Axis:");
    display.drawString(60, 16, Yv);

    //read sensors
    distance_right = ultrasonic_one.read(CM);
    distance_left = ultrasonic_two.read(CM);
    //distance_mid = ultrasonic_four.read(CM);
    distance_mid = ultrasonic_three.read(CM);

    //code for displaying distance variables
    distance_right = ultrasonic_one.read(CM);
    char temp1[50];
    sprintf(temp1, "%d", distance_right);
    const char* c = temp1;

    distance_left = ultrasonic_two.read(CM);
    char temp4[50];
    sprintf(temp4, "%d", distance_left);
    const char* f = temp4;

    distance3 = ultrasonic_three.read(CM);
    char temp5[50];
    sprintf(temp5, "%d", distance3);
    const char* g = temp5;



    //display content
    display.drawString(0, 32, "Dist. (cm):");
    display.drawString(100, 32, c);
    display.drawString(60, 32, f);
    display.drawString(80, 32, g);


    //code for displaying motor variables
    char temp2[50];
    char temp3[50];
    sprintf(temp2, "%d", motor_control_left);
    sprintf(temp3, "%d", motor_control_right);
    const char* d = temp2;
    const char* e = temp3;
    //display content
    display.drawString(0, 48, "Motor Value:");
    display.drawString(80, 48, d);
    display.drawString(100, 48, e);


    display.display();//display on oled

    //clear display to update variables
    display.clear();
}

//make writing to motors easier
void motor_write(int x, int y) {
    motor_control_left = x;
    motor_control_right = y;

    motor_left.write(motor_control_left);//write value to motor for reverse, stop and go
    motor_right.write(motor_control_right);
}

//calculate whether the cross product is positive or negative to determine which way to turn;
int calc_cross(int x_1, int y_1, int x_2, int y_2, int xT, int yT){
  int cross_product = ((x_2-x_1)*(yT-y_2)) - ((y_2-y_1)*(xT-x_2));
  if (cross_product>0){
    Serial.println("Turn Left");
    return 1;
  }

  else if (cross_product<0){
    Serial.println("Turn Right");
    return -1;
  }
}

float angle_calc(int x_1, int y_1, int x_2, int y_2, int xT, int yT) {
    float dot_product, magnitudes, angle;
    dot_product = ((x_2 - x_1) * (xT - x_2)) + ((y_2 - y_1) * (yT - y_2)); //find dot product (numerator)
    magnitudes = hypot(x_2 - x_1, y_2 - y_1) * hypot(xT - x_2, yT - y_2); // multiply the magnitudes (denominator)
    angle = acos(dot_product / magnitudes) * 180 / (2 * acos(0.0)); //change from radians to degree 
    Serial.println(angle);
    return angle;
}


void turn_towards_target() {
    
    float angle;
    int angle_dir;
  
    //calculate angle but not if the positions are grabage
    if (x_1 != -1 && y_1 != -1 && (x_1 != x_2 || y_1 != y_2)) {
        angle = angle_calc(x_1, y_1, x_2, y_2, xT, yT);
        angle_dir = calc_cross(x_1, y_1, x_2, y_2, xT, yT);
    }

    float left_ms_per_deg = 3.06;
    float right_ms_per_deg = 3.19;
    //stop for split second to kill momentum
    motor_write(90, 90);
    delay(100);


    if (angle_dir == 1) {
        float delay_time = angle * left_ms_per_deg;
        motor_write(60, 60);
        delay(delay_time);
    }
    if (angle_dir == -1) {
        float delay_time = angle * right_ms_per_deg;
        motor_write(120, 120);
        delay(delay_time);
    }
}

void check_angle() {
  float angle;
  int angle_dir;
    if (x_1 != -1 && y_1 != -1 && (x_1 != x_2 || y_1 != y_2)) {
        angle = angle_calc(x_1, y_1, x_2, y_2, xT, yT);
        angle_dir = calc_cross(x_1, y_1, x_2, y_2, xT, yT);
    }
    if (angle > 20) {
      throttle = 3;
    }
   /* if (angle < 80 && angle_dir == 1) {
        throttle = 5;
    }
    if (angle < 80 && angle_dir == -1) {
        throttle = 4;
    }
    if (angle > 80) {
        throttle = 3;
    }*/
}
//int count = 0;
void object_avoidance() {
    //check if approaching wall
    if (distance_mid < 7) {
        display_readings();

      motor_write(75,105);
      delay(200);

       int angle_dir;
       /*if (count == 0){
        motor_write(120,120);
        delay(145);
       }else{*/
         if (x_1 != -1 && y_1 != -1 && (x_1 != x_2 || y_1 != y_2)) {
            angle_dir = calc_cross(x_1, y_1, x_2, y_2, xT, yT);
         }
  
        float left_ms_per_deg = 3.06;
        float right_ms_per_deg = 3.19;
         
          //turn 90 degrees away from wall in direction of target
         if (angle_dir == 1) {
            float delay_time = 90 * left_ms_per_deg;
            motor_write(60, 60);
            delay(delay_time);
        }
        else if (angle_dir == -1) {
            float delay_time = 90 * right_ms_per_deg;
            motor_write(120, 120);
            delay(delay_time);
        }
        /*else{
          motor_write(180,180);
          delay(200);
        }*/
        //count=1;
       //}

    }
    else {

        display_readings();

        //keep away from walls
        if (distance_left < 4) {

            throttle = 1;//left side is too close to wall

        }

        if (distance_right < 4) {

            throttle = 2;//right side is too close to wall

        }


    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    // put your setup code here, to run once:

    Serial.begin(115200);

    //wifi stuff
    setup_wifi();
    delay(3000);
    Serial.println("Wemos POWERING UP ......... ");
    client.setServer(mqtt_server, mqtt_port);   //This 1883 is a TCP/IP port number for MQTT 
    client.setCallback(callback);

    //Motor Functions
    motor_left.attach(motor_leftpin);
    motor_right.attach(motor_rightpin);
    //ultrasonic functions


    //display functions
    display.init();
    display.flipScreenVertically();
    display.display();
}

void loop() {

    //subscribe the data from mqtt server
    if (!client.connected()) {
        reconnect();
    }
    display_readings();
    
    xT=xt[tar_arr_pos];
    yT=yt[tar_arr_pos];

    Serial.println(xT);
    Serial.println(yT);

    throttle = 0;//controls movement
    
    
    check_angle();
    object_avoidance();

    switch (throttle) {
    case 1://left side close
        motor_write(140, 90);//slow down one tread turn right

        break;
    case 2://right side too close
        motor_write(90, 40);//slow down one tread turn left
        
        break;
    case 3://stop and turn towards target
        turn_towards_target();
        break;
    case 4:
        motor_write(180, 70);//veer to the right
        break;
    case 5:
        motor_write(110, 0);//veer to the left
        break;
    default:
        motor_write(99, 76);//full speed ahead
        break;

    }

    //check if within range of target and stop for 2 seconds and set next target
    if (hypot(xT - x_2, yT - y_2) < 100) {
      if(tar_arr_pos != 2 && tar_arr_pos != 4){                                           //  DO NOT STOP AT DUMMY TARGETS CODE HERE
        motor_write(90, 90);
        delay(2000);
      }
        tar_arr_pos++;
    }

    //write new values to previous position varibales
    if (x_1 != x_2 || y_1 != y_2) {
        x_1 = x_2;
        y_1 = y_2;
    }
}
