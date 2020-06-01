/*
 * fifo.c
 *
 *  Created on: Apr 10, 2020
 *      Author: Ardoli
 */

#include "fifo.h"

char putbit = 0;
char getbit = 0;


      // FIFO data type
volatile DataType fifo[FIFO_SIZE];  // FIFO storage array
volatile int fifo_head = 0; // index of the first item in the FIFO
volatile int fifo_tail = 0; // index one step past the last item

// put data into the FIFO, skip if full
// returns 1 on success, 0 if FIFO was full
int fifo_put(DataType data)
{
    int new_tail = fifo_tail + 1;
    if (new_tail >= FIFO_SIZE) new_tail = 0; // wrap around
    if (fifo_head != new_tail) {    // if the FIFO is not full
        fifo[fifo_tail] = data;     // store data into the FIFO
        fifo_tail = new_tail;       // advance FIFO tail index
        return 1;                   // success
    }
    return 0;   // full
}

// get data from the FIFO
// returns 1 on success, 0 if FIFO was empty
int fifo_get(DataType *data)
{
    if (fifo_head != fifo_tail) {   // if the FIFO is not empty
        *data = fifo[fifo_head];    // read data from the FIFO
////        IntMasterDisable();
//        fifo_head++;                // advance FIFO head index
////        delay_us(1000);
//        if (fifo_head >= FIFO_SIZE) fifo_head = 0; // wrap around
////        IntMasterEnable();
        if (fifo_head == FIFO_SIZE - 1)
            fifo_head = 0;
        else
            fifo_head++;
        return 1;                   // success
    }
    return 0;   // empty
}



