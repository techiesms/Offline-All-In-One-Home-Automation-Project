/*

 This code is for Offline All in One Home Autoamtion Project made using
 ESP32 Module and VC02 Moudle 

 And this code is for ESP32 Module 

 This code is tested with 
 - Arduino IDE Version 2.3.2
 - ESP32 Boards Package Version 2.0.6

*/


// Necessary Libraries

#include <IRremote.h>   // https://github.com/Arduino-IRremote/Arduino-IRremote (2.6.0)
#include <AceButton.h>  // https://github.com/bxparks/AceButton (1.9.2)
#include <DHT.h>        // https://github.com/adafruit/DHT-sensor-library (1.4.3)
#include "BluetoothSerial.h"
#include <SimpleTimer.h>
#include <Preferences.h>
Preferences pref;


// Bluetooth Configuration
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;


// Timer for Sensors data
SimpleTimer Timer;

// Set your Bluetooth Device Name
String Bluetooth_Device_Name = "ESP32_techiesms";

// Set Time Interval to Send DHT Sensor's Data
uint8_t Send_Sensor_Data_Interval = 10;  // Time in Seconds

// By Default all the Relays will be in OFF State
#define DEFAULT_RELAY_STATE false

using namespace ace_button;

// IR Remote Code for Lights
#define IR_Relay1 0x1FE50AF
#define IR_Relay2 0x1FED827
#define IR_Relay3 0x1FEF807
#define IR_Relay4 0x1FE30CF
#define IR_Relay_All_Off 0x1FE48B7
#define IR_Relay_All_On 0x1FE7887

// IR Remote Code for Fan
#define IR_Speed_Up 0x1FE609F
#define IR_Speed_Dw 0x1FEA05F
#define IR_Fan_off 0x1FE10EF
#define IR_Fan_on 0x1FE906F

//Relay Pins
#define r1 25
#define r2 26
#define r3 4
#define r4 22

//Switch Pins
#define switch1 32
#define switch2 35
#define switch3 34
#define switch4 39
#define fan_switch 33

// Pins of Fan Regulator Knob
#define s1 27
#define s2 14
#define s3 23  //12
#define s4 13

// Pins of Relay (Fan Speed Control)
#define Speed1 21
#define Speed2 19
#define Speed4 18

// Sensor's Pin
#define irPin 17  // IR sensor pin
#define DHTPIN 5  // DHT sensor pin


// HEX Values Coming from VC02 Voice Command
#define Light1ON 0xA1
#define Light1OFF 0xA0
#define Light2ON 0xB1
#define Light2OFF 0xB0
#define Light3ON 0xC1
#define Light3OFF 0xC0
#define Light4ON 0xD1
#define Light4OFF 0xD0
#define FanON 0xFF
#define FanOFF 0xF0
#define FanS1 0xF1
#define FanS2 0xF2
#define FanS3 0xF3
#define FanS4 0xF4


bool RELAY_STATE_FLAG = 0;

bool FeedBack_Speed0 = 1;
bool FeedBack_Speed1 = 1;
bool FeedBack_Speed2 = 1;
bool FeedBack_Speed3 = 1;
bool FeedBack_Speed4 = 1;

// Switch Flags
bool SWITCH5_FLAG = 1;
bool SWITCH6_FLAG = 1;
bool SWITCH7_FLAG = 1;
bool SWITCH8_FLAG = 1;

// Default Relay State
bool relay1 = LOW;
bool relay2 = LOW;
bool relay3 = LOW;
bool relay4 = LOW;

// Relay State
bool switch_state_ch1 = LOW;
bool switch_state_ch2 = LOW;
bool switch_state_ch3 = LOW;
bool switch_state_ch4 = LOW;

// Flags for Fan Speed
bool speed1_flag = 1;
bool speed2_flag = 1;
bool speed3_flag = 1;
bool speed4_flag = 1;
bool speed0_flag = 1;

int curr_speed = 0;

unsigned int receivedValue = 0;
int bluedata;  // variable for storing bluetooth data

int incoming;  // variable to store byte received from phone
int id = -1;
int val_byte1 = -1;
int val_byte2 = -1;

// Relay State
bool toggleState_1 = LOW;  //Define integer to remember the toggle state for relay 1
bool toggleState_2 = LOW;  //Define integer to remember the toggle state for relay 2
bool toggleState_3 = LOW;  //Define integer to remember the toggle state for relay 3
bool toggleState_4 = LOW;  //Define integer to remember the toggle state for relay 4
int FanState = 0;

#define DEBUG_SW 1


// Declaring & Setting Deafult value in all variables
float temperature_value = 0;
float humidity_value = 0;


void Relay_Control(int Relay_Pin, int Relay_state);

DHT dht(DHTPIN, DHT11);
IRrecv irrecv(irPin);
decode_results results;


ButtonConfig config1;
AceButton button1(&config1);
ButtonConfig config2;
AceButton button2(&config2);
ButtonConfig config3;
AceButton button3(&config3);
ButtonConfig config4;
AceButton button4(&config4);
ButtonConfig config5;
AceButton button5(&config5);

void handleEvent1(AceButton*, uint8_t, uint8_t);
void handleEvent2(AceButton*, uint8_t, uint8_t);
void handleEvent3(AceButton*, uint8_t, uint8_t);
void handleEvent4(AceButton*, uint8_t, uint8_t);
void handleEvent5(AceButton*, uint8_t, uint8_t);

void getRelayState() 
{
  toggleState_1 = pref.getBool("Relay1", 0);
  Serial.print("Last State Relay1 - ");
  Serial.println(toggleState_1);
  send_BT(1, toggleState_1);
  digitalWrite(relay1, toggleState_1);
  delay(200);

  toggleState_2 = pref.getBool("Relay2", 0);
  Serial.print("Last State Relay2- ");
  Serial.println(toggleState_2);
  send_BT(2, toggleState_1);
  digitalWrite(relay2, toggleState_2);

  delay(200);

  toggleState_3 = pref.getBool("Relay3", 0);
  Serial.print("Last State Relay3- ");
  Serial.println(toggleState_3);
  digitalWrite(relay3, toggleState_3);
  send_BT(3, toggleState_3);
  delay(200);

  toggleState_4 = pref.getBool("Relay4", 0);
  Serial.print("Last State Relay4- ");
  Serial.println(toggleState_4);
  digitalWrite(relay4, toggleState_4);
  send_BT(4, toggleState_4);
  delay(200);

  FanState = pref.getInt("Fan", 0);
  Serial.print("Last State Fan- ");
  Serial.println(FanState);

  if (FanState == 0) {
    // FeedBack_Speed0 = 1;
    speed_0();
  } else if (FanState == 1) {
    FeedBack_Speed1 = 1;
    speed_1();
  } else if (FanState == 2) {
    FeedBack_Speed2 = 1;
    speed_2();
  } else if (FanState == 3) {
    FeedBack_Speed3 = 1;
    speed_3();
  } else if (FanState == 4) {
    FeedBack_Speed4 = 1;
    speed_4();
  } else {
  }
  delay(200);
}
void UpdateApp() {
  send_BT(10, 1);
  delay(200);

  toggleState_1 = pref.getBool("Relay1", 0);
  Serial.print("Last u State Relay1 - ");
  Serial.println(toggleState_1);
  send_BT(1, toggleState_1);
  delay(200);

  toggleState_2 = pref.getBool("Relay2", 0);
  Serial.print("Last State Relay2- ");
  Serial.println(toggleState_2);
  send_BT(2, toggleState_1);
  delay(200);

  toggleState_3 = pref.getBool("Relay3", 0);
  Serial.print("Last State Relay3- ");
  Serial.println(toggleState_3);
  send_BT(3, toggleState_3);
  delay(200);

  toggleState_4 = pref.getBool("Relay4", 0);
  Serial.print("Last State Relay4- ");
  Serial.println(toggleState_4);
  send_BT(4, toggleState_4);
  delay(200);

  FanState = pref.getInt("Fan", 0);
  Serial.print("Last State Fan- ");
  Serial.println(FanState);
  if (FanState > 0) {
    FanState++;
    send_BT(5, FanState);
  } else if (FanState == 0) {
    send_BT(5, 0);
  }

  send_BT(10, 0);
}


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);                     // For Serial Monitor
  Serial2.begin(9600);                    // For VC02 Module
  SerialBT.begin(Bluetooth_Device_Name);  //Bluetooth device name


  Serial.println("The device started, now you can pair it with bluetooth!");

  pref.begin("Relay_State", false);

  pinMode(switch1, INPUT_PULLUP);
  pinMode(switch2, INPUT_PULLUP);
  pinMode(switch3, INPUT_PULLUP);
  pinMode(switch4, INPUT_PULLUP);
  pinMode(fan_switch, INPUT_PULLUP);

  pinMode(s1, INPUT_PULLUP);
  pinMode(s2, INPUT_PULLUP);
  pinMode(s3, INPUT_PULLUP);
  pinMode(s4, INPUT_PULLUP);

  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  pinMode(r3, OUTPUT);
  pinMode(r4, OUTPUT);
  pinMode(Speed1, OUTPUT);
  pinMode(Speed2, OUTPUT);
  pinMode(Speed4, OUTPUT);

  config1.setEventHandler(button1Handler);
  config2.setEventHandler(button2Handler);
  config3.setEventHandler(button3Handler);
  config4.setEventHandler(button4Handler);
  config5.setEventHandler(button5Handler);

  button1.init(switch1);
  button2.init(switch2);
  button3.init(switch3);
  button4.init(switch4);
  button5.init(fan_switch);

  Timer.setInterval((Send_Sensor_Data_Interval * 1000));

  irrecv.enableIRIn();  // Enabling IR sensor
  dht.begin();      // Beginning DHT Sensor's Data
  getRelayState();  // Get the last state of Relays
}

void loop() 
{
  //********** Cheak APP is Connected and Update ESP'S Current State to APP *******************
  if (SerialBT.hasClient()) {
    // Serial.println("Bluetooth device connected!");
    if (RELAY_STATE_FLAG == 1) {
      Serial.println("Bluetooth device connected!");
      delay(500);
      UpdateApp();  // update the last state of Relays
    }

    RELAY_STATE_FLAG = 0;
  } else {
    // Serial.println("No Bluetooth device connected.");
    RELAY_STATE_FLAG = 1;
  }

  //*********Cheaking "Bluetooth(APP)" is giving any Data************
  if (SerialBT.available()) {
    BluetoothControl();
  }

  //********** Cheaking "VC-02" is giving any Data**********
  if (Serial2.available()) {
    VoiceControl();
  }


  button1.check();
  button2.check();
  button3.check();
  button4.check();
  button5.check();
  fan();

  ir_remote();

  if (Timer.isReady()) 
  {
    send_sensor();  // Send DHT Data to APP
    Timer.reset();  // Reset a timer
  }
}



//********* Loop for controlling "VC-02" incoming data **************
void VoiceControl() {
  int receivedValue = Serial2.read();  // Read the incoming byte
  Serial.print("HEX VALUE - ");
  Serial.println(receivedValue, HEX);  // Print the value in HEX format

  // Print the received value in HEX format
  Serial.print("Received HEX value: 0x");
  Serial.println(receivedValue, HEX);

  //************ Relay Control **************
  if (receivedValue == Light1ON)  // Check if the value is A2 in HEX
  {
    digitalWrite(r1, HIGH);
    send_BT(1, 1);

  } else if (receivedValue == Light1OFF) {
    digitalWrite(r1, LOW);
    send_BT(1, 0);

  } else if (receivedValue == Light2ON) {
    digitalWrite(r2, HIGH);
    send_BT(2, 1);

  } else if (receivedValue == Light2OFF) {
    digitalWrite(r2, LOW);
    send_BT(2, 0);

  } else if (receivedValue == Light3ON) {
    digitalWrite(r3, HIGH);
    send_BT(3, 1);

  } else if (receivedValue == Light3OFF) {
    digitalWrite(r3, LOW);
    send_BT(3, 0);

  } else if (receivedValue == Light4ON) {
    digitalWrite(r4, HIGH);
    send_BT(4, 1);

  } else if (receivedValue == Light4OFF) {
    digitalWrite(r4, LOW);
    send_BT(4, 0);

  }

  //*************FAN Control*******************************
  else if (receivedValue == FanON) {
    speed_1();
    send_BT(5, 1);
  } else if (receivedValue == FanOFF) {
    speed_0();
    send_BT(5, 0);
    Serial.print("FAN OFF\n");
  }

  else if (receivedValue == FanS1) {
    speed_1();

  } else if (receivedValue == FanS2) {
    speed_2();

  } else if (receivedValue == FanS3) {
    speed_3();

  } else if (receivedValue == FanS4) {
    speed_4();

  } else {
  }
  delay(10);
  receivedValue = 0;
}


//************ Loop for controlling "Bluetooth(APP)"" in coming data **************
void BluetoothControl() {

  incoming = SerialBT.read();  //Read what we receive and store in "incoming"

  if (incoming > 127) {
    reset_rx_BT();
    id = incoming - 128;
  } else if (val_byte1 == -1) {
    val_byte1 = incoming;
  } else if (val_byte2 == -1) {
    val_byte2 = incoming;
    int value = 128 * val_byte1 + val_byte2;
    Serial.print("id:");
    Serial.println(id);
    Serial.print("val:");
    Serial.println(value);

    Relay_Control(id, value);

    reset_rx_BT();
  }
}

//***** Will Reset "BT flags" **********
void reset_rx_BT() {
  id = -1;
  val_byte1 = -1;
  val_byte2 = -1;
}

//***** For sending data to "APP" **********
void send_BT(int id, int value) {
  SerialBT.write(128 + id);
  SerialBT.write(floor(value / 128));
  SerialBT.write(value % 128);
}

//**************** Loop for Controlling RELAYS with "IR Remote" **********
void ir_remote() {
  //if (DEBUG_SW)Serial.println("Inside IR REMOTE");
  if (irrecv.decode(&results)) {
    if (DEBUG_SW) Serial.println(results.value, HEX);  //print the HEX code
    switch (results.value) {
      case IR_Relay1:
        switch_state_ch1 = !switch_state_ch1;
        digitalWrite(r1, switch_state_ch1);
        send_BT(1, switch_state_ch1);
        if (DEBUG_SW) Serial.println("RELAY1");
        pref.putBool("Relay1", switch_state_ch1);
        delay(100);
        break;
      case IR_Relay2:
        switch_state_ch2 = !switch_state_ch2;
        digitalWrite(r2, switch_state_ch2);
        send_BT(2, switch_state_ch2);
        if (DEBUG_SW) Serial.println("RELAY2");
        pref.putBool("Relay2", switch_state_ch2);
        delay(100);
        break;
      case IR_Relay3:
        switch_state_ch3 = !switch_state_ch3;
        digitalWrite(r3, switch_state_ch3);
        send_BT(3, switch_state_ch3);
        if (DEBUG_SW) Serial.println("RELAY3");
        pref.putBool("Relay3", switch_state_ch3);
        delay(100);
        break;
      case IR_Relay4:
        switch_state_ch4 = !switch_state_ch4;
        digitalWrite(r4, switch_state_ch4);
        send_BT(4, switch_state_ch4);
        if (DEBUG_SW) Serial.println("RELAY4");
        pref.putBool("Relay4", switch_state_ch4);
        delay(100);
        break;
      case IR_Relay_All_Off:
        All_Lights_Off();
        break;
      case IR_Relay_All_On:
        All_Lights_On();
        break;

        FeedBack_Speed0 = 1;
        FeedBack_Speed1 = 1;
        FeedBack_Speed2 = 1;
        FeedBack_Speed3 = 1;
        FeedBack_Speed4 = 1;

      case IR_Fan_on:
        if (curr_speed == 0) {
          speed_1();

        } else if (curr_speed == 1) {
          speed_1();

        } else if (curr_speed == 2) {
          speed_2();

        } else if (curr_speed == 3) {
          speed_3();

        } else if (curr_speed == 4) {
          speed_4();

        } else {
        }
        break;
      case IR_Fan_off:
        speed_0();
        send_BT(5, 0);
        break;
      case IR_Speed_Up:
        if (curr_speed == 1) {
          speed_2();
        } else if (curr_speed == 2) {
          speed_3();
        } else if (curr_speed == 3) {
          speed_4();
        } else if (curr_speed == 4) {
          //Do nothing
        } else {
        }

        break;
      case IR_Speed_Dw:
        if (curr_speed == 1) {
          //Do nothing
        }
        if (curr_speed == 2) {
          speed_1();
        }
        if (curr_speed == 3) {
          speed_2();
        }
        if (curr_speed == 4) {
          speed_3();
        } else {
        }

        break;
      default: break;
    }
    irrecv.resume();
  }
}

void All_Lights_Off() {
  switch_state_ch1 = 0;
  digitalWrite(r1, LOW);
  send_BT(1, 0);
  pref.putBool("Relay1", switch_state_ch1);

  switch_state_ch2 = 0;
  digitalWrite(r2, LOW);
  send_BT(2, 0);
  pref.putBool("Relay2", switch_state_ch2);

  switch_state_ch3 = 0;
  digitalWrite(r3, LOW);
  send_BT(3, 0);
  pref.putBool("Relay3", switch_state_ch3);

  switch_state_ch4 = 0;
  digitalWrite(r4, LOW);
  send_BT(4, 0);
  pref.putBool("Relay4", switch_state_ch4);
}

void All_Lights_On() {
  switch_state_ch1 = 1;
  digitalWrite(r1, HIGH);
  send_BT(1, 1);
  pref.putBool("Relay1", switch_state_ch1);

  switch_state_ch2 = 1;
  digitalWrite(r2, HIGH);
  send_BT(2, 1);
  pref.putBool("Relay2", switch_state_ch2);

  switch_state_ch3 = 1;
  digitalWrite(r3, HIGH);
  send_BT(3, 1);
  pref.putBool("Relay3", switch_state_ch3);

  switch_state_ch4 = 1;
  digitalWrite(r4, HIGH);
  send_BT(4, 1);
  pref.putBool("Relay4", switch_state_ch4);
}

//functions for defineing manual switch

void button1Handler(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  if (DEBUG_SW) Serial.println("EVENT1");
  switch (eventType) {
    case AceButton::kEventPressed:
      if (DEBUG_SW) Serial.println("kEventPressed");
      switch_state_ch1 = true;
      digitalWrite(r1, HIGH);
      send_BT(1, 1);
      pref.putBool("Relay1", switch_state_ch1);

      break;
    case AceButton::kEventReleased:
      if (DEBUG_SW) Serial.println("kEventReleased");
      switch_state_ch1 = false;
      digitalWrite(r1, LOW);
      send_BT(1, 0);
      pref.putBool("Relay1", switch_state_ch1);
      break;
  }
}
void button2Handler(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  if (DEBUG_SW) Serial.println("EVENT2");

  switch (eventType) {
    case AceButton::kEventPressed:
      if (DEBUG_SW) Serial.println("kEventPressed");
      switch_state_ch2 = true;
      digitalWrite(r2, HIGH);
      send_BT(2, 1);
      pref.putBool("Relay2", switch_state_ch2);
      break;
    case AceButton::kEventReleased:
      if (DEBUG_SW) Serial.println("kEventReleased");
      switch_state_ch2 = false;
      digitalWrite(r2, LOW);
      send_BT(2, 0);
      pref.putBool("Relay2", switch_state_ch2);
      break;
  }
}
void button3Handler(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  if (DEBUG_SW) Serial.println("EVENT3");

  switch (eventType) {
    case AceButton::kEventPressed:
      if (DEBUG_SW) Serial.println("kEventPressed");
      switch_state_ch3 = true;
      digitalWrite(r3, HIGH);
      send_BT(3, 1);
      pref.putBool("Relay3", switch_state_ch3);
      break;
    case AceButton::kEventReleased:
      if (DEBUG_SW) Serial.println("kEventReleased");
      switch_state_ch3 = false;
      digitalWrite(r3, LOW);
      send_BT(3, 0);
      pref.putBool("Relay3", switch_state_ch3);
      break;
  }
}
void button4Handler(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  if (DEBUG_SW) Serial.println("EVENT4");

  switch (eventType) {
    case AceButton::kEventPressed:
      if (DEBUG_SW) Serial.println("kEventPressed");
      switch_state_ch4 = true;
      digitalWrite(r4, HIGH);
      send_BT(4, 1);
      pref.putBool("Relay4", switch_state_ch4);
      break;
    case AceButton::kEventReleased:
      if (DEBUG_SW) Serial.println("kEventReleased");
      switch_state_ch4 = false;
      digitalWrite(r4, LOW);
      send_BT(4, 0);
      pref.putBool("Relay4", switch_state_ch4);
      break;
  }
}

void button5Handler(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  if (DEBUG_SW) Serial.println("EVENT5");
  switch (eventType) {
    case AceButton::kEventPressed:
      if (DEBUG_SW) Serial.println("kEventPressed");

      FeedBack_Speed0 = 1;
      FeedBack_Speed1 = 1;
      FeedBack_Speed2 = 1;
      FeedBack_Speed3 = 1;
      FeedBack_Speed4 = 1;


      if (curr_speed == 0) {
        speed_0();
      }
      if (curr_speed == 1) {
        speed_1();
      }
      if (curr_speed == 2) {
        speed_2();
      }
      if (curr_speed == 3) {
        speed_3();
      }
      if (curr_speed == 4) {
        speed_4();
      }
      break;
    case AceButton::kEventReleased:
      if (DEBUG_SW) Serial.println("kEventReleased");
      digitalWrite(Speed1, LOW);
      digitalWrite(Speed2, LOW);
      digitalWrite(Speed4, LOW);
      // curr_speed = 0;
      send_BT(5, 0);
      break;
  }
}

//******** Cheaking the state of rotary switch ********
void fan() {
  if (digitalRead(s1) == LOW && speed1_flag == 1) {
    speed_1();
    speed1_flag = 0;
    speed2_flag = 1;
    speed3_flag = 1;
    speed4_flag = 1;
    speed0_flag = 1;
  }
  if (digitalRead(s2) == LOW && digitalRead(s3) == HIGH && speed2_flag == 1) {
    speed_2();

    speed1_flag = 1;
    speed2_flag = 0;
    speed3_flag = 1;
    speed4_flag = 1;
    speed0_flag = 1;
  }
  if (digitalRead(s3) == LOW && speed3_flag == 1) {
    speed_3();

    speed1_flag = 1;
    speed2_flag = 1;
    speed3_flag = 0;
    speed4_flag = 1;
    speed0_flag = 1;
  }
  if (digitalRead(s4) == LOW && speed4_flag == 1) {
    speed_4();

    speed1_flag = 1;
    speed2_flag = 1;
    speed3_flag = 1;
    speed4_flag = 0;
    speed0_flag = 1;
  }
  if (digitalRead(s1) == HIGH && digitalRead(s2) == HIGH && digitalRead(s3) == HIGH && digitalRead(s4) == HIGH && speed0_flag == 1) {
    speed_0();
    // send_BT(5, 0);
    speed1_flag = 1;
    speed2_flag = 1;
    speed3_flag = 1;
    speed4_flag = 1;
    speed0_flag = 0;
  }
}

//functions for defineing of speeds

void speed_0() {
  //All Relays Off - Fan at speed 0
  if (DEBUG_SW) Serial.println("SPEED 0");
  curr_speed = 0;
  digitalWrite(Speed1, LOW);
  digitalWrite(Speed2, LOW);
  digitalWrite(Speed4, LOW);

  if (FeedBack_Speed0 == 1) {
    send_BT(5, 0);
    FeedBack_Speed0 = 0;
    FeedBack_Speed1 = 1;
    FeedBack_Speed2 = 1;
    FeedBack_Speed3 = 1;
    FeedBack_Speed4 = 1;
  }

  pref.putInt("Fan", curr_speed);
}

void speed_1() {
  //Speed1 Relay On - Fan at speed 1
  if (DEBUG_SW) Serial.println("SPEED 1");
  curr_speed = 1;
  digitalWrite(Speed1, LOW);
  digitalWrite(Speed2, LOW);
  digitalWrite(Speed4, LOW);
  delay(1000);
  digitalWrite(Speed1, HIGH);

  if (FeedBack_Speed1 == 1) {
    send_BT(5, 2);
    FeedBack_Speed0 = 1;
    FeedBack_Speed1 = 0;
    FeedBack_Speed2 = 1;
    FeedBack_Speed3 = 1;
    FeedBack_Speed4 = 1;
  }

  pref.putInt("Fan", curr_speed);
}

void speed_2() {
  //Speed2 Relay On - Fan at speed 2
  if (DEBUG_SW) Serial.println("SPEED 2");
  curr_speed = 2;
  digitalWrite(Speed1, LOW);
  digitalWrite(Speed2, LOW);
  digitalWrite(Speed4, LOW);
  delay(1000);
  digitalWrite(Speed2, HIGH);

  if (FeedBack_Speed2 == 1) {
    send_BT(5, 3);
    FeedBack_Speed0 = 1;
    FeedBack_Speed1 = 1;
    FeedBack_Speed2 = 0;
    FeedBack_Speed3 = 1;
    FeedBack_Speed4 = 1;
  }

  pref.putInt("Fan", curr_speed);
}

void speed_3() {
  //Speed1 & Speed2 Relays On - Fan at speed 3
  if (DEBUG_SW) Serial.println("SPEED 3");
  curr_speed = 3;
  digitalWrite(Speed1, LOW);
  digitalWrite(Speed2, LOW);
  digitalWrite(Speed4, LOW);
  delay(1000);
  digitalWrite(Speed1, HIGH);
  digitalWrite(Speed2, HIGH);

  if (FeedBack_Speed3 == 1) {
    send_BT(5, 4);
    FeedBack_Speed0 = 1;
    FeedBack_Speed1 = 1;
    FeedBack_Speed2 = 1;
    FeedBack_Speed3 = 0;
    FeedBack_Speed4 = 1;
  }

  pref.putInt("Fan", curr_speed);
}

void speed_4() {
  //Speed4 Relay On - Fan at speed 4
  if (DEBUG_SW) Serial.println("SPEED 4");
  curr_speed = 4;
  digitalWrite(Speed1, LOW);
  digitalWrite(Speed2, LOW);
  digitalWrite(Speed4, LOW);
  delay(1000);
  digitalWrite(Speed4, HIGH);

  if (FeedBack_Speed4 == 1) {
    send_BT(5, 5);
    FeedBack_Speed0 = 1;
    FeedBack_Speed1 = 1;
    FeedBack_Speed2 = 1;
    FeedBack_Speed3 = 1;
    FeedBack_Speed4 = 0;
  }

  pref.putInt("Fan", curr_speed);
}

//*********** DHT sensor data processing and Updating it to "Bluetooth(APP)" **************
void send_sensor() {

  float h = dht.readHumidity();
  float t = dht.readTemperature();  // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    if (DEBUG_SW) Serial.println("Failed to read from DHT sensor!");
    send_BT(8, 1);
    Serial.print("e");
    return;
  } else {

    send_BT(6, t);
    delay(500);
    send_BT(7, h);
    delay(500);

    if (DEBUG_SW) Serial.print("Temperature - ");
    if (DEBUG_SW) Serial.println(t);
    if (DEBUG_SW) Serial.print("Humidity - ");
    if (DEBUG_SW) Serial.println(h);
  }
}

//********** Relay Control via Bluetooth(App) **************

void Relay_Control(int Relay_Pin, int Relay_state) {
  int _Pin = Relay_Pin;
  int _State = Relay_state;

  if (_Pin == 1) {
    if (_State == 1) {
      switch_state_ch1 = 1;
      digitalWrite(r1, HIGH);

      pref.putBool("Relay1", switch_state_ch1);
    } else {
      switch_state_ch1 = 0;
      digitalWrite(r1, LOW);
      pref.putBool("Relay1", switch_state_ch1);
    }
  }

  if (_Pin == 2) {
    if (_State == 1) {
      switch_state_ch2 = 1;
      digitalWrite(r2, HIGH);
      pref.putBool("Relay2", switch_state_ch2);
    } else {
      switch_state_ch2 = 0;
      digitalWrite(r2, LOW);
      pref.putBool("Relay2", switch_state_ch2);
    }
  }

  if (_Pin == 3) {
    if (_State == 1) {
      switch_state_ch3 = 1;
      digitalWrite(r3, HIGH);
      pref.putBool("Relay3", switch_state_ch3);
    } else {
      switch_state_ch3 = 0;
      digitalWrite(r3, LOW);
      pref.putBool("Relay3", switch_state_ch3);
    }
  }

  if (_Pin == 4) {
    if (_State == 1) {
      switch_state_ch4 = 1;
      digitalWrite(r4, HIGH);
      pref.putBool("Relay4", switch_state_ch4);
    } else {
      switch_state_ch4 = 0;
      digitalWrite(r4, LOW);
      pref.putBool("Relay4", switch_state_ch4);
    }
  }
  //*********Fan Switch***********
  if (_Pin == 5) {
    if (_State == 1) {
      if (curr_speed == 0) {
        curr_speed = 0;
        digitalWrite(Speed1, LOW);
        digitalWrite(Speed2, LOW);
        digitalWrite(Speed4, LOW);

        pref.putInt("Fan", curr_speed);
      }
      if (curr_speed == 1) {
        curr_speed = 1;
        digitalWrite(Speed1, LOW);
        digitalWrite(Speed2, LOW);
        digitalWrite(Speed4, LOW);
        delay(1000);
        digitalWrite(Speed1, HIGH);

        pref.putInt("Fan", curr_speed);
      }
      if (curr_speed == 2) {
        curr_speed = 2;
        digitalWrite(Speed1, LOW);
        digitalWrite(Speed2, LOW);
        digitalWrite(Speed4, LOW);
        delay(1000);
        digitalWrite(Speed2, HIGH);

        pref.putInt("Fan", curr_speed);
      }
      if (curr_speed == 3) {
        curr_speed = 3;
        digitalWrite(Speed1, LOW);
        digitalWrite(Speed2, LOW);
        digitalWrite(Speed4, LOW);
        delay(1000);
        digitalWrite(Speed1, HIGH);
        digitalWrite(Speed2, HIGH);

        pref.putInt("Fan", curr_speed);
      }
      if (curr_speed == 4) {
        curr_speed = 4;
        digitalWrite(Speed1, LOW);
        digitalWrite(Speed2, LOW);
        digitalWrite(Speed4, LOW);
        delay(1000);
        digitalWrite(Speed4, HIGH);

        pref.putInt("Fan", curr_speed);
      }
    }
    if (_State == 0) {
      digitalWrite(Speed1, LOW);
      digitalWrite(Speed2, LOW);
      digitalWrite(Speed4, LOW);
    }

    //***************Fan Speed***************
    if (_State == 2) {
      curr_speed = 1;
      digitalWrite(Speed1, LOW);
      digitalWrite(Speed2, LOW);
      digitalWrite(Speed4, LOW);
      delay(1000);
      digitalWrite(Speed1, HIGH);

      pref.putInt("Fan", curr_speed);
    } else if (_State == 3) {
      curr_speed = 2;
      digitalWrite(Speed1, LOW);
      digitalWrite(Speed2, LOW);
      digitalWrite(Speed4, LOW);
      delay(1000);
      digitalWrite(Speed2, HIGH);

      pref.putInt("Fan", curr_speed);
    } else if (_State == 4) {
      curr_speed = 3;
      digitalWrite(Speed1, LOW);
      digitalWrite(Speed2, LOW);
      digitalWrite(Speed4, LOW);
      delay(1000);
      digitalWrite(Speed1, HIGH);
      digitalWrite(Speed2, HIGH);

      pref.putInt("Fan", curr_speed);
    } else if (_State == 5) {
      curr_speed = 4;
      digitalWrite(Speed1, LOW);
      digitalWrite(Speed2, LOW);
      digitalWrite(Speed4, LOW);
      delay(1000);
      digitalWrite(Speed4, HIGH);

      pref.putInt("Fan", curr_speed);
    }

    else {
    }
  }
}