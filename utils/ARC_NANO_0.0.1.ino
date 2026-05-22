// for reference use only

#include <Wire.h>
#define SLAVE_ADDRESS 0x08
// extra four bytes just in case
byte data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// no enable pins because they are connected directly to 5V
int LPWM_PIN_A = 6;
int RPWM_PIN_A = 5;

int LPWM_PIN_B = 10;
int RPWM_PIN_B = 9;

void setup() 
{
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  pinMode(LPWM_PIN_A, OUTPUT);
  pinMode(RPWM_PIN_A, OUTPUT);
  pinMode(LPWM_PIN_B, OUTPUT);
  pinMode(RPWM_PIN_B, OUTPUT);
}
void loop() {
    // byte 0: forward A, byte 1: backward A, byte 2: forward B, byte 3: backward B
    int PWM_F_A = (int) data[0];
    int PWM_B_A = (int) data[1];
    int PWM_F_B = (int) data[2];
    int PWM_B_B = (int) data[3];

    // where speed is 0 - 255
    if (PWM_F_A > 0) {
        analogWrite(LPWM_PIN_A, 0);
        analogWrite(RPWM_PIN_A, PWM_F_A);
    } else {
        // forward
        analogWrite(LPWM_PIN_A, PWM_B_A);
        analogWrite(RPWM_PIN_A, 0);
    }

    if (PWM_F_B > 0) {
        analogWrite(LPWM_PIN_B, 0);
        analogWrite(RPWM_PIN_B, PWM_F_B);
    } else {
        // forward
        analogWrite(LPWM_PIN_B, PWM_B_B);
        analogWrite(RPWM_PIN_B, 0);
    }
}
void receiveData(int bytecount)
{
  for (int i = 0; i < bytecount; i++) {
    data[i] = Wire.read();
  }
}
void sendData()
{
  for (int i = 0; i < sizeof(data); i++) {
    Wire.write(data[i]);
  }
}
