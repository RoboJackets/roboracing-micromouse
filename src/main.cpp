#include <Arduino.h>
#include <cmath>
#include "pins.h"
#include "constants.h"

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(EMIT_1, OUTPUT);
  pinMode(EMIT_2, OUTPUT);
  pinMode(EMIT_3, OUTPUT);
  pinMode(EMIT_4, OUTPUT);

  pinMode(RECV_1, INPUT);
  pinMode(RECV_2, INPUT);
  pinMode(RECV_3, INPUT);
  pinMode(RECV_4, INPUT);

  Serial.begin(9600);
}

void loop() {
  int pre = analogRead(RECV_1);
  digitalWrite(EMIT_1, HIGH);
  delayMicroseconds(EMIT_RECV_DELAY_US);
  int post = analogRead(RECV_1);
  digitalWrite(EMIT_1, LOW);

  double dist = 0.647426 / pow(max(post, 1), 0.516999);

  Serial.print(">sensor_pre:");
  Serial.println(pre);
  Serial.print(">sensor_post:");
  Serial.println(post);
  Serial.print(">dist:");
  Serial.println(dist);
  delay(100);
}