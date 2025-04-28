#define IMAGE_H
#include <stdio.h>

#define IMAGE_SIZE 128
#define WORLD_SIZE 4000
typedef unsigned char BYTE;

void read_pbm(char *filename, BYTE **img);
