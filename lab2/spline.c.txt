#include "eyebot.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define INTERVAL 0.02   // split the whole track into 50 incremental pieces
#define FILENAME "way.txt"
#define MAX_POINTS 20
#define SHRINK_RATIO 10
#define SPEED 50

void LCDCircleAlign(int ax, int ay) {
    int lcd_x, lcd_y;
    LCDGetSize(&lcd_x, &lcd_y);
    // printf("LCD Circle: %d %d\n", ax, ay);
    // Center and also shrink the track to fit the LCD screen
    int x = (ax / SHRINK_RATIO) + (lcd_x / 2);
    int y = (lcd_y / 2) - (ay / SHRINK_RATIO);
    LCDCircle(x, y, 5, RED, 1);
}

// Start spline drive at a given x/y position and angle
void SplineDrive(int x, int y, int alpha) {
    int ax, ay, rphi;
    VWGetPosition(&ax, &ay, &rphi);
    LCDCircleAlign(ax, ay); 
    // printf("(%d, %d) %d\n", ax, ay, rphi);

    int bx = x;
    int by = y;

    int x_now = ax;
    int y_now = ay;

    float len = 2 * sqrt(pow(x, 2) + pow(y, 2));
    float Dax = len;
    float Day = 0;
    float Dbx = len * cos(alpha * M_PI / 180);
    float Dby = len * sin(alpha * M_PI / 180);
    
    for (float u = 0; u < 1 + INTERVAL; u += INTERVAL) {
        int rx, ry;
        VWGetPosition(&rx, &ry, &rphi);
        LCDCircleAlign(rx, ry);
        // printf("(%d, %d) %d\n", rx, ry, rphi);

        float u2 = pow(u, 2);
        float u3 = pow(u, 3);

        float h0 = 2 * u3 - 3 * u2 + 1;
        float h1 = -2 * u3 + 3 * u2;
        float h2 = u3 - 2 * u2 + u;
        float h3 = u3 - u2;

        float sp_x = ax * h0 + bx * h1 + Dax * h2 + Dbx * h3;
        float sp_y = ay * h0 + by * h1 + Day * h2 + Dby * h3;

        int sphi = round(atan2(sp_y - y_now, sp_x - x_now) * 180 / M_PI);
        float distance = len / (2 * floor(1 / INTERVAL));

        VWCurve(distance, sphi - rphi, SPEED);
        VWWait();

        x_now = sp_x;
        y_now = sp_y;
    }

    VWGetPosition(&ax, &ay, &rphi);
    LCDCircleAlign(ax, ay);
    printf("Sprint Drive (%d, %d) %d finished!\n", x, y, alpha);
}

typedef struct {
    int x;
    int y;
} Point;

int main(int argc, char *argv[]) {
    LCDMenu("Set", "Task1", "Task2", "END");
    
    while (1) {
        int key = KEYRead();
        if (key == KEY1) {
            SIMSetRobot(0, 2000, 2000, 0, 0);
            VWSetPosition(0, 0, 0);
        } else if (key == KEY2) {
            SplineDrive(0, 2000, 90);
        } else if (key == KEY3) {
            FILE *way = fopen(FILENAME, "r");
            if (way) {
                Point points[MAX_POINTS];
                int num_points = 0;
                while (num_points < MAX_POINTS && fscanf(way, "%d %d", &points[num_points].x, &points[num_points].y) == 2) {
                    num_points++;
                }
                fclose(way);
                
                if (num_points > 0) {
                    printf("Way No.0 (To Start Pos): Increments: %d %d\n", points[0].x, points[0].y);
                    SplineDrive(points[0].x, points[0].y, 0);
                    OSWait(1000);

                    int prev_x = points[0].x;
                    int prev_y = points[0].y;

                    int i = 1;
                    while (1) {
                        if (i >= num_points) i = 0;
                        printf("Way No.%d: Increments: %d %d\n", i+1, points[i].x - prev_x, points[i].y - prev_y);
                        SplineDrive(points[i].x - prev_x, points[i].y - prev_y, 0);
                        OSWait(1000);
                        prev_x = points[i].x;
                        prev_y = points[i].y;
                        i++;
                    }
                }
            }
        } else if (key == KEY4) {
            break;
        } else {
            OSWait(100);
        }
    }
    return 0;
}
