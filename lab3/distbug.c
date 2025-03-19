#include "eyebot.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define INITIAL_X 500
#define INITIAL_Y 500
// #define DRIVING_SPEED 100
#define DRIVING_SPEED 100
#define TURNING_SPEED 50
// #define SAFE_DISTANCE 200
// #define ABSOLUTE_SAFE_DISTANCE 150
#define SAFE_DISTANCE 250
#define ABSOLUTE_SAFE_DISTANCE 200
#define RIGHT_ANGLE 90
#define TURNING_ANGLE 20
#define MATH_INF 9999
#define STEP 500
#define EPSILON 10

#define LIDAR_FRONT_LEFT 150
#define LIDAR_FRONT 180
#define LIDAR_FRONT_RIGHT 210
#define LIDAR_RIGHT 270
#define LIDAR_LESS_RIGHT 260
#define LIDAR_MORE_RIGHT 280
#define GOAL_TOLERANCE 500

typedef struct {
    int x;
    int y;
} Position;

typedef enum {
    DRIVING,
    ROTATING,
    FOLLOWING
} State;

// int INITIAL_ANGLE = 0;
int euclidean_distance(int dx, int dy) {
    return (int)sqrt(dx * dx + dy * dy);
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Usage: ./distbug.x x y goal position (relative) wrt to rot initial position. 2600, -3100 | 2100 -2200 \n");
        return EXIT_FAILURE;
    }
    
    Position GOAL = {atoi(argv[1]), atoi(argv[2])};
    // INITIAL_ANGLE = atoi(argv[3]);
    LCDMenu("Set", "Start", "", "END");
    // KEYWait(KEY1);
    // SIMSetRobot(0, INITIAL_X, INITIAL_Y, 0, INITIAL_ANGLE);
    // VWSetPosition(INITIAL_X, INITIAL_Y, INITIAL_ANGLE);
    
    KEYWait(KEY2);
    int dists[360];
    int x, y, phi;
    State state = DRIVING;    // TODO leave obstacle condition
    VWSetSpeed(DRIVING_SPEED, 0);
    Position hit;
    // int left_hit_point = 0;
    int d_min = MATH_INF;
    int prev;
    int counter = 0;
    while (1) {
        VWGetPosition(&x, &y, &phi);
        Position curr = {x, y};
        int dx = GOAL.x - curr.x;
        int dy = GOAL.y - curr.y;
        float theta = atan2(dy, dx) * 180 / M_PI;
        if (theta > 180) theta -= 360;
        float diff = round(theta - phi);
        LCDSetPrintf(1, 0, "dx: %d, dy: %d, theta: %.2f, diff: %.2f\n", dx, dy, theta, diff);

        // Goal found
        if (abs(dx) < 50 && abs(dy) < 50) {
            printf("Goal found\n");
            VWSetSpeed(0, 0);
            return EXIT_SUCCESS;
        }
        
        LIDARGet(dists);
        switch (state) {
            case DRIVING:
                if (dists[LIDAR_FRONT] < SAFE_DISTANCE ||
                    dists[LIDAR_FRONT_LEFT] < ABSOLUTE_SAFE_DISTANCE ||
                    dists[LIDAR_FRONT_RIGHT] < ABSOLUTE_SAFE_DISTANCE) {
                    printf("DRIVE: Start rotating\n");
                    VWSetSpeed(0, 0);
                    state = ROTATING;
                    hit = curr; // Obstacle hit point
                    counter = 0;
                } else if (abs(diff) >= 1.0) {
                    printf("DRIVE: GO CURVE diff %f\n", diff);
                    VWSetSpeed(DRIVING_SPEED, (int)diff);
                } else {
                    printf("DRIVE: GO STRAIGHT abs(diff) %f\n", diff);
                    VWSetSpeed(DRIVING_SPEED, 0);
                }
                break;
            
            case ROTATING:
                // diff = round(phi - 90)  // TODO perp?
                // if (abs(diff) > 5) {
                //     VWSetSpeed(0, 50);
                // } else {
                //     VWSetSpeed(0, 0);
                //     state = FOLLOWING;
                // }



                // Origin turn right
                // VWTurn(TURNING_ANGLE, TURNING_SPEED);
                // VWWait();

                // Approximately perpendicular to wall
                if (dists[LIDAR_RIGHT] < SAFE_DISTANCE &&
                    dists[LIDAR_MORE_RIGHT] >= dists[LIDAR_RIGHT] &&
                    dists[LIDAR_LESS_RIGHT] >= dists[LIDAR_RIGHT]) {
                    printf("ROTATING: Start following\n");
                    VWSetSpeed(TURNING_SPEED, 0);
                    // VWSetSpeed(0, 0);
                    state = FOLLOWING;
                } else {
                    printf("ROTATING: GO ROTATING\n");
                    VWSetSpeed(0, TURNING_ANGLE);  // TODO Turn right
                }
                break;
            
            case FOLLOWING:
                counter++;
                if (counter > 10 && abs(hit.x - GOAL.x) < GOAL_TOLERANCE && abs(hit.y - GOAL.y) < GOAL_TOLERANCE) {
                    printf("Goal unreachable\n");
                    VWSetSpeed(0, 0);
                    return EXIT_FAILURE;
                }
                // if (!left_hit_point && (hit.x != curr.x || hit.y != curr.y)) {
                //     left_hit_point = 1;
                // } else if (left_hit_point && (hit.x == curr.x && hit.y == curr.y)) {
                //     VWSetSpeed(0, 0);
                //     printf("Goal unreachable\n");
                //     return EXIT_FAILURE;
                // }
                prev = dists[LIDAR_RIGHT];
                OSWait(100);
                LIDARGet(dists);
                if (dists[LIDAR_RIGHT] > SAFE_DISTANCE + 150) {
                    // VWTurn(-TURNING_ANGLE, TURNING_SPEED);
                    // VWWait();
                    // VWStraight(25, TURNING_SPEED);
                    // VWWait();
                    printf("FOLLOWING: Start rotating\n");
                    state = ROTATING;
                } else if (dists[LIDAR_FRONT] < SAFE_DISTANCE) {
                    // VWTurn(RIGHT_ANGLE, TURNING_SPEED);
                    // VWWait();
                    // VWStraight(25, TURNING_SPEED);
                    // VWWait();
                    printf("FOLLOWING: Start rotating\n");
                    state = ROTATING;
                } else if (dists[LIDAR_RIGHT] < 100 ||
                           dists[LIDAR_FRONT_RIGHT] < ABSOLUTE_SAFE_DISTANCE) {
                    // VWTurn(TURNING_ANGLE, TURNING_SPEED);
                    // VWWait();
                    // VWStraight(25, TURNING_SPEED);
                    // VWWait();
                    printf("FOLLOWING: Start rotating\n");
                    state = ROTATING;
                } else if (dists[LIDAR_RIGHT] - prev > EPSILON ||
                           dists[LIDAR_RIGHT] > SAFE_DISTANCE ||
                           dists[LIDAR_MORE_RIGHT] <= dists[LIDAR_RIGHT]) {
                    printf("FOLLOWING: GO CURVE\n");
                    VWSetSpeed(TURNING_SPEED, -TURNING_ANGLE);
                } else if (prev - dists[LIDAR_RIGHT] > EPSILON ||
                           dists[LIDAR_RIGHT] < ABSOLUTE_SAFE_DISTANCE ||
                           dists[LIDAR_LESS_RIGHT] <= dists[LIDAR_RIGHT]) {
                    printf("FOLLOWING: GO CURVE\n");
                    VWSetSpeed(TURNING_SPEED, TURNING_ANGLE);
                }
                
                int d = euclidean_distance(dx, dy);
                d_min = d < d_min ? d : d_min;
                int angle = 180 - (theta - phi);
                if (angle < 0) angle += 360; // TODO += 360
                int f = dists[angle];   // free space to goal
                
                // Check leave condition
                if (d - f <= d_min - STEP) {
                    LCDPrintf("Leaving obstacle angle %d d %d, f %d, d_min %d, STEP %d \n", angle, d, f, d_min, STEP);
                    // printf("Leaving obstacle\n");
                    VWSetSpeed(0, 0);
                    VWStraight(300, DRIVING_SPEED);
                    VWWait();

                    diff = round(theta - phi);
                    VWTurn(diff, TURNING_SPEED);
                    VWWait();
                    state = DRIVING;

                    // TODO Reset hit point
                    // left_hit_point = 0;
                    d_min = MATH_INF;
                }
                break;
        }
    }
    
    printf("end\n");
    return EXIT_SUCCESS;
}
