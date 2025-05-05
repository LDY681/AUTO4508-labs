#include "eyebot.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#define LCD_WIDTH 320
#define LCD_HEIGHT 240
#define SCALE 0.05f             // 1 mm = 0.05 LCD pixel
#define LIDAR_POINTS 360
#define LIDAR_RANGE 360

#define FORWARD_SPEED 100       // mm/s
#define TURN_SPEED  100         // deg/s
#define SAFE_DISTANCE 300       // mm, threshold for obstacle detection

#define DEG_TO_RAD (M_PI / 180.0f)

// Store initial robot pose to fix the LCD origin in world space
int origin_x, origin_y, origin_phi;

void draw_map(int *distances) {
    int robot_x, robot_y, robot_phi;
    VWGetPosition(&robot_x, &robot_y, &robot_phi);
    float robot_heading = (robot_phi - origin_phi) * DEG_TO_RAD;

    for (int i = 0; i < LIDAR_POINTS; i += 2) { // Reduce density for performance
        int d = distances[i];
        if (d <= 0 || d > 3000) continue; // skip invalid or too far points

        float angle = robot_heading + i * DEG_TO_RAD;
        float wx = robot_x + d * cosf(angle);
        float wy = robot_y + d * sinf(angle);

        int lcd_x = (int)((wx - origin_x) * SCALE);
        int lcd_y = (int)((wy - origin_y) * SCALE);

        // Flip y-axis for LCD coordinate system
        lcd_y = LCD_HEIGHT - lcd_y;

        if (lcd_x >= 0 && lcd_x < LCD_WIDTH && lcd_y >= 0 && lcd_y < LCD_HEIGHT) {
            LCDPixel(lcd_x, lcd_y, WHITE);
        }
    }
}

int front_blocked(int *distances) {
    for (int i = 170; i <= 190; i++) {
        if (distances[i] > 0 && distances[i] < SAFE_DISTANCE)
            return 1;
    }
    return 0;
}

int main() {
    LCDClear();
    LCDMenu("START", "", "", "END");
    VWGetPosition(&origin_x, &origin_y, &origin_phi);

    KEYWait(KEY1);

    int dists[LIDAR_POINTS];

    while (1) {
        int x, y, phi;
        VWGetPosition(&x, &y, &phi);
        LCDSetPrintf(0, 0, "x=%d y=%d phi=%d ", x, y, phi);

        LIDARGet(dists);
        draw_map(dists);

        if (front_blocked(dists)) {
            VWSetSpeed(0, 0);
            int turn_angle = 180 + (rand() % 180); // turn between 180-360
            VWTurn(turn_angle, TURN_SPEED);
            VWWait();
        } else {
            int rand_angle = (rand() % 60) - 30; // -30 to 30 deg/s
            VWSetSpeed(FORWARD_SPEED, rand_angle);
        }
    }

    VWSetSpeed(0, 0);
    return 0;
}