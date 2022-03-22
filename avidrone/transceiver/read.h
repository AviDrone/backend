// Avidrone 2022
#ifndef AVIDRONE_TRANSCEIVER_READ_H_
#define AVIDRONE_TRANSCEIVER_READ_H_

#include <stdbool.h>

#define NUM_READS 100

static volatile int interrupt_count = -1;
static volatile bool SETUP_COMPLETE = false;
static volatile bool all_dig1_reading[NUM_READS][7] = { 0 };
static volatile bool all_dig2_reading[NUM_READS][7] = { 0 };
static volatile bool all_dir_reading[NUM_READS][5] = { 0 };
static volatile bool all_dp_reading[NUM_READS] = { 0 };
static char all_dig_decode[NUM_READS][4];
static double all_dir_dig_out[NUM_READS][2];

// {Dig1, Dig2, Dir} these are the BCM numberings

const int ANODE_PINS[] = { 2, 3, 4 };
const int CATH_PINS[] = { 17, 27, 22, 5, 10, 11, 9 }; // {a, b, c, d, e, f, g}
const int DIR_PINS[] = { 10, 5, 22, 27, 17 }; // {L_most, L, C, R, R_most}
const int DP_PIN = 0;
const bool ZERO[] = { 0, 0, 0, 0, 0, 0, 1 };
const bool ONE[] = { 1, 0, 0, 1, 1, 1, 1 };
const bool TWO[] = { 0, 0, 1, 0, 0, 1, 0 };
const bool THREE[] = { 0, 0, 0, 0, 1, 1, 0 };
const bool FOUR[] = { 1, 0, 0, 1, 1, 0, 0 };
const bool FIVE[] = { 0, 1, 0, 0, 1, 0, 0 };
const bool SIX[] = { 0, 1, 0, 0, 0, 0, 0 };
const bool SEVEN[] = { 0, 0, 0, 1, 1, 1, 1 };
const bool EIGHT[] = { 0, 0, 0, 0, 0, 0, 0 };
const bool NINE[] = { 0, 0, 0, 0, 1, 0, 0 };

bool arrayEqual(bool read_data[7], const bool number[7]);
char decode_7seg(bool cath_7seg[7]);

#endif  // AVIDRONE_TRANSCEIVER_READ_H_

