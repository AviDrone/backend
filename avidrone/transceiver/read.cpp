#include "read.h"
#include <iostream>
#include <fstream>


bool arrayEqual(bool read_data[7], const bool number[7]) {
    int count = 0;
    for (int i = 0; i < 7; i++) {
        if (read_data[i] == number[i]) { count++;}
    }
    if (count == 7) { return true; }
    else { return false; }
}

char decode_7seg(bool cath_7seg[7]) {
    if (arrayEqual(cath_7seg, ZERO)) { return '0'; }
    else if (arrayEqual(cath_7seg, ONE)) { return '1'; }
    else if (arrayEqual(cath_7seg, TWO)) { return '2'; }
    else if (arrayEqual(cath_7seg, THREE)) { return '3'; }
    else if (arrayEqual(cath_7seg, FOUR)) { return '4'; }
    else if (arrayEqual(cath_7seg, FIVE)) { return '5'; }
    else if (arrayEqual(cath_7seg, SIX)) { return '6'; }
    else if (arrayEqual(cath_7seg, SEVEN)) { return '7'; }
    else if (arrayEqual(cath_7seg, EIGHT)) { return '8'; }
    else if (arrayEqual(cath_7seg, NINE)) { return '9'; }
    else { return 'x';
    }
}
