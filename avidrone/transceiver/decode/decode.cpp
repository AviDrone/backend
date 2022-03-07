#include "decode.h"
#include <fstream>
#include <iostream>
#include <time.h>

#define PIN_INPUT 23
#define NUM_READ 10000

int main(int argc, char **argv) {
    clock_t start, end;
    double cpu_time_used;

    
    std::ofstream fout;
    fout.open("/run/user/1000/data_output");

    start = clock();
    for (int i = 0; i < NUM_READ; i++) {
        // fout << digitalRead(PIN_INPUT) << ",\n";
    }
    end = clock();
    fout.close();

    std::cout << "READING FROM GPIO AND WRITING TO RAM \n";
    std::cout << "Number of reads: " << NUM_READ << "\n";

    cpu_time_used = (static_cast<double>(end - start)) / CLOCKS_PER_SEC;
    std::cout << "Total time: " << cpu_time_used << "\n";

    double avg_time_read = cpu_time_used / NUM_READ;
    std::cout << "Average time per read: " << avg_time_read << "\n";
}