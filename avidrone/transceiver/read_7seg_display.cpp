// Copyright 2022 Avidrone

#include "read.h"
#include "read_7seg_display.h"  // Only runs on Raspberry Pi
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
  double dirdig[2] = {1000, 1000};
  while (true) {
    get_dir_dig(dirdig);
    std::cout << dirdig[0] << "__" << dirdig[1] << "\n";
  }
}

bool arrayEqual(bool read_data[7], const bool number[7]) {
  int count = 0;
  for (int i = 0; i < 7; i++) {
    if (read_data[i] == number[i]) {
      count++;
    }
  }
  if (count == 7) {
    return true;
  } else {
    return false;
  }
}

char decode_7seg(bool cath_7seg[7]) {
  if (arrayEqual(cath_7seg, ZERO)) {
    return '0';
  } else if (arrayEqual(cath_7seg, ONE)) {
    return '1';
  } else if (arrayEqual(cath_7seg, TWO)) {
    return '2';
  } else if (arrayEqual(cath_7seg, THREE)) {
    return '3';
  } else if (arrayEqual(cath_7seg, FOUR)) {
    return '4';
  } else if (arrayEqual(cath_7seg, FIVE)) {
    return '5';
  } else if (arrayEqual(cath_7seg, SIX)) {
    return '6';
  } else if (arrayEqual(cath_7seg, SEVEN)) {
    return '7';
  } else if (arrayEqual(cath_7seg, EIGHT)) {
    return '8';
  } else if (arrayEqual(cath_7seg, NINE)) {
    return '9';
  } else {
    return 'x';
  }
}

void int_a_dig1(void) {
  if (SETUP_COMPLETE == true) {
    interrupt_count++;
    if (interrupt_count < NUM_READS) {
      // for (int i = 0; i < 7;i++)  {
      // all_dig1_reading[interrupt_count][i] = digitalRead(CATH_PINS[i]);
      //}
      all_dig1_reading[interrupt_count][0] =
        digitalRead(CATH_PINS[0]);
      all_dig1_reading[interrupt_count][1] =
        digitalRead(CATH_PINS[1]);
      all_dig1_reading[interrupt_count][2] =
        digitalRead(CATH_PINS[2]);
      all_dig1_reading[interrupt_count][3] =
        digitalRead(CATH_PINS[3]);
      all_dig1_reading[interrupt_count][4] =
        digitalRead(CATH_PINS[4]);
      all_dig1_reading[interrupt_count][5] =
        digitalRead(CATH_PINS[5]);
      all_dig1_reading[interrupt_count][6] =
        digitalRead(CATH_PINS[6]);
      all_dig1_reading[interrupt_count][7] =
        digitalRead(CATH_PINS[7]);
      all_dp_reading[interrupt_count] =
        digitalRead((DP_PIN));
    }
  }
}

// Interrupt Handler for the anode of digit two LEDs
void int_a_dig2(void) {
  if ((interrupt_count != -1) && (interrupt_count < NUM_READS) &&
      (SETUP_COMPLETE == true)) {
    for (int i = 0; i < 7; i++) {
      all_dig2_reading[interrupt_count][i] = digitalRead(CATH_PINS[i]);
    }
  }
}
// Interrupt Handler for the anode of the direction LEDs
void int_a_dir(void) {
  if ((interrupt_count != -1) && (interrupt_count < NUM_READS) &&
      (SETUP_COMPLETE == true)) {
    all_dir_reading[interrupt_count][0] =
      digitalRead(DIR_PINS[0]);
    all_dir_reading[interrupt_count][1] =
      digitalRead(DIR_PINS[1]);
    all_dir_reading[interrupt_count][2] =
      digitalRead(DIR_PINS[2]);
    all_dir_reading[interrupt_count][3] =
      digitalRead(DIR_PINS[3]);
    all_dir_reading[interrupt_count][4] =
      digitalRead(DIR_PINS[4]);
  }
}

// Takes bool array of length 5 representing the direction LEDs
// formatted as such, {Leftmost, Left, Center, Right, Rightmost}
// Returns a double from 0-4 where zero is returned when the leftmost
// led lights is on and 4 is returned when the rightmost LED is on.
double dir_decode(bool read_dir[5]) {
  if (
    ((read_dir[0] == 0) && (read_dir[1] == 0)) || (read_dir[0] == 0)
  ) {
    return 0;
  } else if (
    ((read_dir[2] == 0) && (read_dir[1] == 0)) || (read_dir[1] == 0)
  ) {
    return 1;
  } else if (
    ((read_dir[2] == 0) && (read_dir[3] == 0)) || (read_dir[3] == 0)
  ) {
    return 3;
  } else if (
    ((read_dir[4] == 0) && (read_dir[3] == 0)) || (read_dir[4] == 0)
  ) {
    return 4;
  } else if (read_dir[2] == 0) {
    return 2;
  } else {
    return -1;
  }
}

// Function to return double value of distance
// Uses char dig[3] and int dir and returns array
// of doubles with direction then distance.

double dist_to_double(char dist[4]) {
  if ((dist[0] == 'x') && (dist[1] == 'x') && (dist[2] == 'x')) {  // xxx
    return -1;
  } else if ((dist[1] == 'x') && (dist[2] == 'x')) {  // rxx
    return -1;
  } else if ((dist[0] == 'x') && (dist[2] == 'x')) {  // xrx
    return -1;
  } else if ((dist[0] == 'x') && (dist[1] == 'x')) {  // xxr
    return -1;
  } else if ((dist[0] != 'x') && (dist[1] == '.') && (dist[2] == 'x')) {  // rrx
    return -1;
  } else if (
    (dist[0] != 'x') && (dist[1] == 'x') && (dist[2] != 'x')
  ) {  // rxr
    dist[1] = dist[2];
    dist[2] = 'x';
    return atof(dist);
  } else if ((dist[0] != 'x') && (dist[1] == '.') && (dist[2] != 'x')) {  // rrr
    return atof(dist);
  } else if ((dist[0] == 'x') && (dist[1] == '.') && (dist[2] != 'x')) {  // xrr
    dist[0] = '0';
    return atof(dist);
  } else {
    return -1;
  }
}

void get_dir_dig(double *dir_dig_out) {
  struct timespec ts_sleep;
  wiringPiSetupGpio();
  for (int i = 0; i < 7; i++) {
    pinMode(CATH_PINS[i], INPUT);
  }
  for (int i = 0; i < 3; i++) {
    pinMode(ANODE_PINS[i], INPUT);
  }
  pinMode(DP_PIN, INPUT);

  wiringPiISR(ANODE_PINS[0], INT_EDGE_FALLING, &int_a_dig1);
  wiringPiISR(ANODE_PINS[1], INT_EDGE_FALLING, &int_a_dig2);
  wiringPiISR(ANODE_PINS[2], INT_EDGE_FALLING, &int_a_dir);
  SETUP_COMPLETE = true;
  // Wait until 10 reads have occurred or more than 1ms has passed
  while (interrupt_count < NUM_READS) {
    ts_sleep.tv_sec = 0;
    ts_sleep.tv_nsec = 250000000;
    nanosleep(&ts_sleep, NULL);
  }
  // Perform operations to determine the reading and filter out bad readings
  for (int i = 0; i < NUM_READS; i++) {
    all_dig_decode[i][0] =
      decode_7seg(reinterpret_cast<bool *>(all_dig1_reading[i]));
    all_dig_decode[i][2] =
      decode_7seg(reinterpret_cast<bool *>(all_dig2_reading[i]));
    all_dig_decode[i][3] = '\0';
    if (all_dp_reading[i] == 0) {
      all_dig_decode[i][1] = '.';
    } else {
      all_dig_decode[i][1] = all_dig_decode[i][2];
      all_dig_decode[i][2] = '\0';
    }
    all_dir_dig_out[i][1] = dist_to_double(all_dig_decode[i]);
    all_dir_dig_out[i][0] = dir_decode(
      reinterpret_cast<bool *>(all_dir_reading[i])
    );
  }

  // Directional LEDs 0-4 and then -1 if not read
  // int dir_counts[6] = {0};
  int dir_counts[5] = {0};
  for (int i = 0; i < NUM_READS; i++) {
    double count_dir = all_dir_dig_out[i][0];
    // if (count_dir == -1) {
    //    dir_counts[5]++;
    //} else {
    //    dir_counts[(int)count_dir]++;
    //}
    if (count_dir != -1) {
      dir_counts[(static_cast<int>(count_dir)]++;
    }
  }
  int max_index = 0;
  for (int i = 1; i < 5; i++) {
    if (dir_counts[i] > dir_counts[max_index]) {
      max_index = i;
    }
  }

  // Compute the average of all the distance
  // measurements disregarding all '-1'
  double distance_total = 0;
  for (int i = 0; i < NUM_READS; i++) {
    double distance_read = all_dir_dig_out[i][1];
    if (distance_read != -1) {
      distance_total += all_dir_dig_out[i][1];
    }
  }

  dir_dig_out[1] = distance_total / NUM_READS;
  dir_dig_out[0] = max_index;

  // if (max_index == 5) {
  //    dir_dig_out[0] = -1;
  //} else {
  //    dir_dig_out[0] = max_index;
  //}
  interrupt_count = -1;
}
