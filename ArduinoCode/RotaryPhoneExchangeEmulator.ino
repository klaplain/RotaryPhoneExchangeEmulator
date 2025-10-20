#include <HardwareSerial.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>  // include the library

hw_timer_t *Timer_2Hz = NULL, *Timer_20Hz = NULL, *Timer_3kHz = NULL, *Debounce_Timer = NULL, *Idle_Timer = NULL, *Timer_HangUp = NULL;
HardwareSerial myDFPlayer(1);  // Use UART1 for DFPlayer
uint8_t current_phone_state;
volatile uint8_t rm_counter = 0;  // Ring sound counter
uint8_t last_SHK;                 // last measured SHK value
unsigned long last_SHK_low_to_high = millis();
unsigned long last_SHK_high_to_low = millis();
byte playing_state;
unsigned int number_to_be_played;
unsigned int IR_Preset[] = { 211, 212, 213, 214, 215, 216, 217, 311, 312 };

// Pin Definitions
#define SHK 35   // Switch Hook Pin
#define FR 39    // Forward Reverse Pin
#define RM 37    // Ringing Mode Pin
#define TEST 16  // Test Output Pin
#define RXD2 7   // Serial input from DFPlayer
#define TXD2 5   // Serial output to DFPlayer
#define BUSY 3   // DFPlayer Busy  - Needs to change to 9 for PCB

// On/off hook statea\s
#define SHK_HIGH 1  // Denotes off hook = Receiver picked up
#define SHK_LOW 0   // Denotes on hook = Receiver put down

// Critical Timings
#define LOW_PULSE_DIAL_THRESHOLD 50               // ms low dial pulse as 100
#define HIGH_PULSE_DIAL_THRESHOLD 50              // ms high dial pulse was 100
#define HIGH_PULSE_END_DIAL_THRESHOLD 600          // ms end of dialing digit
#define HIGH_PULSE_END_DIAL_NUMBER_THRESHOLD 5000  // ms end of dialing number
#define PICKUP_THRESHOLD 600
#define HANGUP_THRESHOLD 1000  // ms how long before we accept the user has hung up

// Sound file locations and standard tones
#define MASTER_DIRECTORY 99  // Directory number for standard tones
#define MUSIC_DIRECTORY 2
#define DIAL_TONE 1
#define RING_TONE 2
#define DISCONNECT_TONE 3
#define THIS_NUMBER_HAS_BEEN_DISCONNECTED 4

// dfPlayer return status / state
#define NOT_READY 0x3F
#define FINISHED_PLAYING_SOUND 0x3D
#define PLAYING_SOUND 0x41
#define NO_FILE_FOUND 0x40

enum PhoneState { IDLE,
                  READY_TO_DIAL,
                  DIALING,
                  WAITING_FOR_PICKUP,
                  RINGING,
                  PLAYING,
                  END_OF_PLAY };

void ARDUINO_ISR_ATTR on2Hz_Timer() {
  rm_counter = (rm_counter + 1) % 16;
  if ((rm_counter == 0 || rm_counter == 1 || rm_counter == 3 || rm_counter == 4) && (digitalRead(SHK) == 0))
    digitalWrite(RM, HIGH);
  else
    digitalWrite(RM, LOW);  // RM
}

void ARDUINO_ISR_ATTR on20Hz_Timer() {  // For making ring sound
  if (digitalRead(RM) == HIGH)
    digitalWrite(FR, !digitalRead(FR));  //FR
  else
    digitalWrite(FR, LOW);
}

void ARDUINO_ISR_ATTR onHangUp_Timer() {  // We have been on hook for long enough to justify a hangup
  stop_sound();                           //  Our HangUp timer has timed out so reset everything
  ESP.restart();
}

void ARDUINO_ISR_ATTR onSHK_Change() {  // SHK has changed
  if (digitalRead(SHK) == HIGH) {       // User picked up receiver and is now off hook
    timerWrite(Timer_HangUp, 0);        // Turn off Timer
    timerStop(Timer_HangUp);
  } else {  // User replaced receiver so start counting how long it has been down
    timerStart(Timer_HangUp);
  }
}

void ring_on() {  // Ring the phone
  // return;         // Commented out to keep things silent
  Timer_2Hz = timerBegin(1000);
  timerAttachInterrupt(Timer_2Hz, &on2Hz_Timer);
  timerAlarm(Timer_2Hz, 200, true, 0);

  Timer_20Hz = timerBegin(1000);
  timerAttachInterrupt(Timer_20Hz, &on20Hz_Timer);
  timerAlarm(Timer_20Hz, 25, true, 0);
}

void ring_off() {  // Stop ring the phone
  return;          // Commented out to keep things silent
  timerEnd(Timer_2Hz);
  timerEnd(Timer_20Hz);
}

void stop_sound() {
  sendDFCommand(0x0E, 99, 1);  // stop sound
}

void sendDFCommand(byte Command, byte hb, byte lb) {  // Send command to DFPlayer
  byte commandData[10];                               //This holds all the command data to be sent
  byte q;

  int checkSum;
  commandData[0] = 0x7E;     //Start of new command
  commandData[1] = 0xFF;     //Version information
  commandData[2] = 0x06;     //Data length (not including parity) or the start and version
  commandData[3] = Command;  //The command that was sent through
  commandData[4] = 0x01;     //1 = feedback  0 = no feedback
  commandData[5] = hb;       //High byte of the data sent over
  commandData[6] = lb;       //low byte of the data sent over
  checkSum = -(commandData[1] + commandData[2] + commandData[3] + commandData[4] + commandData[5] + commandData[6]);
  commandData[7] = highByte(checkSum);  //High byte of the checkSum
  commandData[8] = lowByte(checkSum);   //low byte of the checkSum
  commandData[9] = 0xEF;                //End bit

  // int i;
  // for (i = 0; i < 10; i++) {
  //   Serial.print(commandData[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  myDFPlayer.write(commandData, 10);  //  Send it all to DFPlayer
}

byte getDFResponse() {  // Get response from DFPlayer
  boolean got_returned_data = false;
  byte this_char;
  int data_index_pointer = 0;
  byte data_read[8];
  byte latest_code;

  while (got_returned_data == false) {
    if (myDFPlayer.available() > 0) {
      this_char = myDFPlayer.read();
      // Serial.print(this_char, HEX);
      // Serial.print(" ");
      data_read[data_index_pointer] = this_char;
      data_index_pointer = (data_index_pointer + 1) % 8;
      if (this_char == 0xEF) {
        // Serial.println();
        got_returned_data = true;
        latest_code = data_read[(data_index_pointer + 1) % 8];
        return (latest_code);
      }
    }
  }
}

void play_sound_file(byte directory, byte myfile) {
  // Serial.print("Playing ");
  // Serial.print(directory);
  // Serial.print(":");
  // Serial.println(myfile);
  sendDFCommand(byte(0x0F), directory, myfile);
}

int get_dialed_number() {

  bool waiting_for_click = true;
  bool waiting_for_digit = true;
  bool waiting_for_number = true;
  uint8_t dialed_digit = 0;
  uint8_t latest_dialed_number[3];
  uint8_t latest_dialed_number_index = 0;
  unsigned long dialed_number = 0;
  unsigned long duration;

  while (waiting_for_number) {
    if (digitalRead(SHK) == HIGH) {
      if (last_SHK == LOW) {
        // this must be a LOW to HIGH transition.
        last_SHK = HIGH;
        last_SHK_low_to_high = millis();
      } else {
        // SHK continues to be HIGH
        duration = millis() - last_SHK_low_to_high;
        if (duration > HIGH_PULSE_DIAL_THRESHOLD) {
          waiting_for_click = true;  // We've been high long enough.  Now we need to wait for it to go low
        }
        if (duration > HIGH_PULSE_END_DIAL_THRESHOLD) {  // we've been high long enough that we're at the end of the digit
          if (dialed_digit != 0) {
            latest_dialed_number[latest_dialed_number_index++] = dialed_digit;
            dialed_digit = 0;
          }
          if (latest_dialed_number_index == 3) {
            return (100 * latest_dialed_number[0] + 10 * latest_dialed_number[1] + latest_dialed_number[2]);
          }
        }
      }
    } else {
      if (last_SHK == HIGH) {
        // this must be a HIGH to LOW transition
        last_SHK = LOW;
        last_SHK_high_to_low = millis();
        if (digitalRead(BUSY) == LOW) {
          stop_sound();  // stop dial tone
        }
      } else {
        // SHK continues to be LOW
        duration = millis() - last_SHK_high_to_low;
        if (duration > LOW_PULSE_DIAL_THRESHOLD) {
          if (waiting_for_click) {
            dialed_digit++;
            {
            }
            waiting_for_click = false;
          }
        }
        if (duration > HANGUP_THRESHOLD) {
          // Serial.println("User has hung up");
          stop_sound();
          dialed_number = 0;
          waiting_for_click = false;
          waiting_for_number = true;  // reset the flag so we'll drop out of the while loop
        }
      }
    }
  }
}

bool hung_up() {
  if ((last_SHK == HIGH) && (digitalRead(SHK) == LOW)) {  // this must be a HIGH to LOW transition i.e. user put down handset or bounce
    last_SHK = LOW;
    last_SHK_high_to_low = millis();
  }
  if ((last_SHK == LOW) && (digitalRead(SHK) == LOW) && (millis() - last_SHK_high_to_low > HANGUP_THRESHOLD))  // this is still low but has it been low long enough?
    return true;
  else
    return false;
}

void setup() {
  //  Initialize GPIO Pins
  pinMode(SHK, INPUT_PULLUP);
  pinMode(FR, OUTPUT);
  pinMode(RM, OUTPUT);
  pinMode(BUSY, INPUT_PULLUP);
  pinMode(TEST, OUTPUT);  // Used for testing only

  // Initialize serial ports
  Serial.begin(115200);
  myDFPlayer.begin(9600, SERIAL_8N1, RXD2, TXD2);  //start the softwareSerial to DF player
                                                   // while (!Serial)
  ;                                                // Wait until serial initializes

  Serial.println();
  Serial.println("PHONE EXCHANGE STARTING");

  // Initialize DFPlayer
  sendDFCommand(0x0c, 0, 0);  // Reset player
  while (getDFResponse() != NOT_READY) {};
  Serial.println();

  sendDFCommand(0x06, 0, 20);  // Set volume to 15
  while (getDFResponse() != PLAYING_SOUND) {};





  // Initialize phone status
  while (digitalRead(SHK) != 0) {}  //but wait until user replaces receiver
  last_SHK = 1;
  last_SHK_low_to_high = millis();
  last_SHK_high_to_low = millis();

  // initialise state machine
  current_phone_state = IDLE;

  //  Initialize IR REceiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  //  Hang Up interrupt
  attachInterrupt(digitalPinToInterrupt(SHK), onSHK_Change, CHANGE);
  //  Hang Up Timer 600ms
  Timer_HangUp = timerBegin(1000);
  timerAttachInterrupt(Timer_HangUp, &onHangUp_Timer);
  timerAlarm(Timer_HangUp, 600, false, 0);
  timerStop(Timer_HangUp);
}

void loop() {
  char data[6];  // 5 bytes + null terminator
  unsigned int this_dialed_number;




  switch (current_phone_state) {
    case IDLE:
      if ((last_SHK == LOW) && (digitalRead(SHK) == HIGH)) {  // this must be a LOW to HIGH transition i.e. user picked up handset
        if (last_SHK_low_to_high - millis() > PICKUP_THRESHOLD) {
          current_phone_state = READY_TO_DIAL;
        }
        last_SHK = HIGH;
        last_SHK_low_to_high = millis();
      }
      if ((last_SHK == HIGH) && (digitalRead(SHK) == LOW)) {  // this must be a HIGH to LOW transition i.e. user put down handset
        last_SHK = LOW;
        last_SHK_high_to_low = millis();
      }

      // See if we've got an IR signal
      if (IrReceiver.decode()) {
        /*
         * Print a summary of received data
         */
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
          Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
          // We have an unknown protocol here, print extended info
          // IrReceiver.printIRResultRawFormatted(&Serial, true);

          IrReceiver.resume();  // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
        } else {
          IrReceiver.resume();  // Early enable receiving of the next IR frame
          if (IrReceiver.decodedIRData.address == 0x1) {
            Serial.print("Got a command digit ");
            Serial.println(IrReceiver.decodedIRData.command);
            current_phone_state = RINGING;
            number_to_be_played = IR_Preset[IrReceiver.decodedIRData.command];
            ring_on();
          }
        }
        /*
         * Finally, check the received data and perform actions according to the received command
         */
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
          Serial.println(F("Repeat received. Here you can repeat the same action as before."));
        } else {
          if (IrReceiver.decodedIRData.command == 0x10) {
            Serial.println(F("Received command 0x10."));
            // do something
          } else if (IrReceiver.decodedIRData.command == 0x11) {
            Serial.println(F("Received command 0x11."));
            // do something else
          }
        }
      }
      break;

    case READY_TO_DIAL:
      Serial.println("READY TO DIAL");
      play_sound_file(MASTER_DIRECTORY, DIAL_TONE);  // play dial tone
      current_phone_state = DIALING;
      break;

    case DIALING:
      Serial.println("DIALING ");
      this_dialed_number = get_dialed_number();
      if (this_dialed_number != 0) {
        Serial.print("DIALING: ");
        Serial.println(this_dialed_number);
        number_to_be_played = this_dialed_number;
        current_phone_state = WAITING_FOR_PICKUP;
      } else {
        current_phone_state = END_OF_PLAY;  // bad numnber
        Serial.println("Bad Number.  Going End of PLay");
      }
      break;

    case WAITING_FOR_PICKUP:
      play_sound_file(MASTER_DIRECTORY, RING_TONE);  // play ringing tone
      delay(5000);
      current_phone_state = PLAYING;
      Serial.println("PLAYING");
      // Serial.print(int(number_to_be_played / 100));
      // Serial.print(":");
      // Serial.println(number_to_be_played % 100);
      play_sound_file(int(number_to_be_played / 100), (number_to_be_played % 100));  // 99:5 is a music sound file
      break;

    case PLAYING:
      playing_state = getDFResponse();  // The problem is that this is blocking until another message comes in.  We need to arrange it so that if it is playing, we check to see whether the user has hung up
      if (playing_state == NO_FILE_FOUND) {
        current_phone_state = END_OF_PLAY;
        delay(2000);                                                           // wiat for a bit
        play_sound_file(MASTER_DIRECTORY, THIS_NUMBER_HAS_BEEN_DISCONNECTED);  // play disconnect tone  -   Please redial - number not known
        Serial.println("Going to END OF PLAY");
      }
      if (playing_state == FINISHED_PLAYING_SOUND) {
        current_phone_state = END_OF_PLAY;
        delay(2000);                                         // wiat for a bit
        play_sound_file(MASTER_DIRECTORY, DISCONNECT_TONE);  // play disconnect tone  -   Please redial - number not known
        Serial.println("Going to END OF PLAY");
      }
      break;

    case END_OF_PLAY:
      Serial.println("Going to IDLE");
      current_phone_state = IDLE;
      break;

    case RINGING:
      Serial.println("RINGING");
      while (digitalRead(SHK) == LOW) {}  // Wait until user picks up
      ring_off();
      delay(600);  // provide a small period until sound starts
      current_phone_state = PLAYING;
      Serial.println("PLAYING");
      play_sound_file(int(number_to_be_played / 100), (number_to_be_played % 100));  // 99:5 is a music sound file
      break;
  }
}
