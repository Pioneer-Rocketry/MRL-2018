#include <math.h>

#define MAX_PASCALS 24485.86

#define MIN_VOLTS 0.25

#define MAX_VOLTS 3.3

#define AIR_DENSITY 1.225

int reading_0 = 0;

int reading_1 = 0;

int reading_2 = 0;

int reading_3 = 0;

int reading_4 = 0;


int pin = A14;

void setup() {
  // put your setup code here, to run once:

  pinMode(pin, INPUT);

  Serial.begin(57600);

}


void loop() {
  // put your main code here, to run repeatedly:

  int min_reading = (int)((float)MIN_VOLTS * 1024.0f/3.3f);

  int max_reading = (int)((float)MAX_VOLTS * 1024.0f/3.3f); 

  float reading_to_pascal = MAX_PASCALS / (max_reading - min_reading);

  int cur_reading = analogRead(pin);

  reading_4 = reading_3;

  reading_3 = reading_2;

  reading_2 = reading_1;

  reading_1 = reading_0;

  reading_0 = cur_reading;

  int smoothed_reading = (reading_0 + 2 * reading_1 + 3 * reading_2 + 2 * reading_3 + reading_4)/9;

  double pascals = (smoothed_reading - min_reading) * reading_to_pascal;

  if(pascals < 0)
    pascals = 0;

  double velocity = sqrt((2 *pascals)/AIR_DENSITY);

  Serial.println(velocity);

}
