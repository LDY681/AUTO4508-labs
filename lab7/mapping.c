#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "eyebot.h"

#define WORLD_SIZE 2000
#define SCALE 0.1f
#define LCD_WIDTH 200
#define LCD_HEIGHT 200

#define LIDAR_POINTS 360
#define LIDAR_RANGE 360

#define FORWARD_SPEED 100
#define TURN_SPEED 100
#define SAFE_DISTANCE 200

#define DEG_TO_RAD (M_PI / 180.0f)
#define SIM_ORIGIN_X 400
#define SIM_ORIGIN_Y 400

void draw_map(int *distances) {
    // LCDClear();
    int rel_x, rel_y, rel_phi;
    VWGetPosition(&rel_x, &rel_y, &rel_phi);

    // Compute absolute position in sim world
    int robot_x = SIM_ORIGIN_X + rel_x;
    int robot_y = SIM_ORIGIN_Y + rel_y;
    float robot_heading = rel_phi * DEG_TO_RAD;

    int robot_px = (int)(robot_x * SCALE);
    int robot_py = LCD_HEIGHT - (int)(robot_y * SCALE);

    for (int i = 0; i < LIDAR_POINTS; i += 1) {  // Reduce density for performance
        int d = distances[i];
        printf("Distance[%d]: %d\n", i, d);
        if (d <= 0 || d > 3000) continue;  // skip invalid or too far points

        float lidar_angle = (i - 180) * DEG_TO_RAD;
        float angle = robot_heading - lidar_angle;
        printf("Angle: %f\n", angle);
        // float angle = robot_heading + i * DEG_TO_RAD;
        float wx = robot_x + d * cosf(angle);
        float wy = robot_y + d * sinf(angle);

        int lcd_x = (int)(wx * SCALE);
        int lcd_y = LCD_HEIGHT - (int)(wy * SCALE);

        LCDLine(robot_px, robot_py, lcd_x, lcd_y, WHITE);
    }
}

void explore() {
    int dists[LIDAR_POINTS];

    while (1) {
        int x, y, phi;
        VWGetPosition(&x, &y, &phi);
        LCDSetPrintf(0, 0, "x=%d y=%d phi=%d ", x, y, phi);

        LIDARGet(dists);
        draw_map(dists);

        int turn_angle = 0;
        if (dists[180] < SAFE_DISTANCE) {
            VWSetSpeed(0, 0);
            turn_angle = 220 + (rand() % 100);  // turn around
            VWTurn(turn_angle, TURN_SPEED);
            VWWait();
        } else if (dists[90] < SAFE_DISTANCE) {  // Left side blocked
            VWSetSpeed(0, 0);
            turn_angle = 70 + (rand() % 40);  // turn around
            VWTurn(-turn_angle, TURN_SPEED);  // Turn right
            VWWait();
        } else if (dists[270] < SAFE_DISTANCE) {  // Right side blocked
            VWSetSpeed(0, 0);
            turn_angle = 70 + (rand() % 40);  // turn around
            VWTurn(turn_angle, TURN_SPEED);  // Turn left
            VWWait();
        } else {
            int rand_angle = (rand() % 60) - 30;  // -30 to 30 deg/s
            VWSetSpeed(FORWARD_SPEED, rand_angle);
        }
    }
}

int main() {
    LCDClear();
    LCDMenu("SET", "START", "TEST", "END");

    int lcd_width = 320;
    int lcd_height = 240;
    double scale = lcd_width / WORLD_SIZE;

    while (1) {
        int key = KEYRead();
        if (key == KEY1) {
            SIMSetRobot(0, SIM_ORIGIN_X, SIM_ORIGIN_Y, 0, 0);
            VWSetPosition(0, 0, 0);
        } else if (key == KEY2) {
            explore();
        } else if (key == KEY3) {
            LCDCircle(100, 100, 5, RED, 1);
            LCDCircle(200, 200, 5, GREEN, 1);
            LCDGetSize(&lcd_width, &lcd_height);
            scale = (lcd_width > lcd_height ? lcd_width : lcd_height) / WORLD_SIZE;
            LCDSetPrintf(0, 0, "Width: %d Height: %d Scale: %d\n", lcd_width, lcd_height, scale);
        }
    }
}
