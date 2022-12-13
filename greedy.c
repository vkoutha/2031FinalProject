#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>

#define XPOS_MASK 0xFFFF000000
#define XPOS_SHIFT 24
#define YPOS_MASK 0xFFFF00
#define YPOS_SHIFT 8
#define DEST_NUM_MASK 0xFF

uint8_t currDestination = 0;
uint64_t destinations[13]; //Format: 0x AAAA (X) BBBB (Y) CC (Dest #) = AAAABBBBCC
uint8_t destinationsOrder[13];
int16_t adjMatrix[13][13];
int16_t adjMatrixAng[13][13];
uint16_t visitedSet;
uint8_t NUM_DESTINATIONS = 0;

void setDestinationPoint(int16_t x, int16_t y, uint8_t destNum);
void fillMatrix();
void printMatrix();
void printRoute();
void createRoute();

int main() {
    currDestination = 0;
    visitedSet = 1;
    setDestinationPoint(0, 0, 0);
    setDestinationPoint(24, 0, 1);
    setDestinationPoint(36, -12, 2);
    setDestinationPoint(48, 0, 3);
    setDestinationPoint(60, 0, 3);
    fillMatrix();
    printMatrix();
    createRoute();
    printRoute();
    return 0;
}

void setDestinationPoint(int16_t x, int16_t y, uint8_t destNum) {
    destinations[destNum] = (x << 24) | (y << 8) | (destNum);
    // 0x FFFF FFFF FF
    //printf("Hex Format: %.10X\n", destinations[destNum]);
    NUM_DESTINATIONS++;
}

void fillMatrix() {
    for(int i = 0; i < NUM_DESTINATIONS; i++) {
        for(int j = 0; j < NUM_DESTINATIONS; j++) {
            int16_t x1pos = (destinations[i] & XPOS_MASK) >> XPOS_SHIFT;
            int16_t x2pos = (destinations[j] & XPOS_MASK) >> XPOS_SHIFT;
            int16_t y1pos = (destinations[i] & YPOS_MASK) >> YPOS_SHIFT;
            int16_t y2pos = (destinations[j] & YPOS_MASK) >> YPOS_SHIFT;
            int16_t xoffset = x2pos - x1pos;
            int16_t yoffset = y2pos - y1pos;
            uint16_t dist = sqrt(pow(xoffset, 2) + pow(yoffset, 2));
            int16_t angle = atan2(yoffset, xoffset) * 180 / M_PI;
            adjMatrix[i][j] = dist;
            adjMatrixAng[i][j] = angle;
        }
    }
}

void createRoute() {
    int closestIdx = -1;
    for(int i = 0; i < NUM_DESTINATIONS - 1; i++) {
        int closest = 100000;
        for(int j = 0; j < NUM_DESTINATIONS; j++) {
            if(j != currDestination) {
                if(adjMatrix[currDestination][j] < closest) {
                    if((visitedSet & (1 << j)) == 0) {
                        closest = adjMatrix[currDestination][j];
                        closestIdx = j;
                        destinationsOrder[i] = closestIdx;
                    }
                }
            }
        }
        visitedSet |= (1 << closestIdx);
        currDestination = closestIdx;
    }
}

/**
 * @brief Outputs AdjMatrix in hex form and then in readable form of "Distance / Angle" 
 * 
 */
void printMatrix() {
    printf("\n");
    for(int i = 0; i < NUM_DESTINATIONS; i++) {
        for(int j = 0; j < NUM_DESTINATIONS; j++) {
            printf("%08X ", adjMatrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");
    for(int i = 0; i < NUM_DESTINATIONS; i++) {
        for(int j = 0; j < NUM_DESTINATIONS; j++) {
                printf("%d / %d   ", adjMatrix[i][j], adjMatrixAng[i][j]);
            }
            printf("\n");
    }
    printf("\n");
}

void printRoute() {
    printf("0 ---> ");
    for(int i = 0; i < NUM_DESTINATIONS - 1; i++) {
        printf("%d ---> ", destinationsOrder[i]);
    }
    printf("Route Completed");
}