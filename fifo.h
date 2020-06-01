/*
 * fifo.h
 *
 *  Created on: Apr 10, 2020
 *      Author: Ardoli
 */

#ifndef FIFO_H_
#define FIFO_H_

#include <stdlib.h>
#include <stdio.h>

#define FIFO_SIZE 11        // FIFO capacity is 1 item fewer
typedef unsigned char DataType;
extern char getbit;
extern char putbit;

int fifo_put(DataType data);
int fifo_get(DataType *data);

#endif /* FIFO_H_ */
