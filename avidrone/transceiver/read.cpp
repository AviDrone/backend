#include "read.h"
#include <iostream>
#include <fstream>

static volatile int interrupt_count = -1;
static volatile bool SETUP_COMPLETE = false;
static volatile bool all_dig1_reading[NUM_READS][7] = {0};
static volatile bool all_dig2_reading[NUM_READS][7] = {0};
static volatile bool all_dir_reading[NUM_READS][5] = {0};
static volatile bool all_dp_reading[NUM_READS] = {0};
static char all_dig_decode[NUM_READS][4];
static double all_dir_dig_out[NUM_READS][2];

// {Dig1, Dig2, Dir} these are the BCM numberings
const int ANODE_PINS[] = {2, 3, 4};
const int CATH_PINS[] = {17, 27, 22, 5, 10, 11, 9}; // {a, b, c, d, e, f, g}
const int DIR_PINS[] = {10, 5, 22, 27, 17};  // {L_most, L, C, R, R_most}
const int DP_PIN = 0;
const bool ZERO[] = {0, 0, 0, 0, 0, 0, 1};
const bool ONE[] = {1, 0, 0, 1, 1, 1, 1};
const bool TWO[] = {0, 0, 1, 0, 0, 1, 0};
const bool THREE[] = {0, 0, 0, 0, 1, 1, 0};
const bool FOUR[] = {1, 0, 0, 1, 1, 0, 0};
const bool FIVE[] = {0, 1, 0, 0, 1, 0, 0};
const bool SIX[] = {0, 1, 0, 0, 0, 0, 0};
const bool SEVEN[] = {0, 0, 0, 1, 1, 1, 1};
const bool EIGHT[] = {0, 0, 0, 0, 0, 0, 0};
const bool NINE[] = {0, 0, 0, 0, 1, 0, 0};

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