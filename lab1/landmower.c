#include "eyebot.h"
#include "../helper.h"
#include <math.h>

#define CORNOR_BUFFER 150

void turn(int *lt_dist, int *rt_dist, int *ft_dist) {
  PSDGetAll(lt_dist, rt_dist, ft_dist);
  int x, y, phi;
  VWGetPosition(&x, &y, &phi);
  printf("X:%4d; Y:%4d; P:%4d\n", x, y, phi);

  printf("Left/Right: (%d, %d) Front: %d\n", *lt_dist, *rt_dist, *ft_dist);

	// LCDPrintf("Turning right align with the wall\n");
	// double angle;
  // angle = atan2((double) *lt_dist,(double) *rt_dist) * 180 / M_PI;
	// if (*lt_dist > *rt_dist) {
  //   // Turn clockwise
  //   angle = -(180 - angle);
	// 	VWTurn(angle, 100);
	// } else {
  //   angle = -angle;
  //   printf("angle: %f\n", angle);
  //   // Turn counter clockwise
	// 	VWTurn(-angle, 100);
	// }
	VWWait();
}

int main() {
  int lt_dist, rt_dist, ft_dist; // left, right and front distance
  LCDPrintf("Lab 1 P1: Landmower\n");

  LCDMenu("START", "", "", "END");
	KEYWait(KEY1);

  // Drive Straight till hit corner
  while(1) {
    VWStraight(10, 100);
    if (PSDGet(PSD_FRONT) < CORNOR_BUFFER) {
      VWSetSpeed(0,0);
      LCDPrintf("Cornor found, stopping the vehicle...\n");
      break;
    };  // STOP if obstacle in front
  }
  turn(&lt_dist, &rt_dist, &ft_dist);
}

	// int count = 1;
	// while (true) {
	// 	LCDPrintf("Drive straight to a wall\n");
	// 	while (PSDGet(PSD_FRONT) > SAFE_DISTANCE) align();

	// 	LCDPrintf("Take a U-turn\n");
	// 	// even count negative angle (clockwise)
	// 	int direction = (count % 2 ? -1 : 1);
	// 	STOP;
	// 	VWTurn(RIGHT_ANGLE * direction, ADJUSTING_SPEED);
	// 	VWWait();

	// 	if (PSDGet(PSD_FRONT) < SAFE_DISTANCE) {
	// 		LCDPrintf("Stop\n");
  //   		KEYWait(KEY4);
  //           return 0;
	// 	}

// 		// drive forward 200 (4 * 50) every align drives forward 50
// 		for (std::size_t i = 0; i < 4; ++i) align();

// 		VWTurn(RIGHT_ANGLE * direction, ADJUSTING_SPEED);
// 		VWWait();
// 		++count;
// 	}
// 	return 0;
// }

// void align() {
// 	int prev, curr;
// 	prev = PSDGet(PSD_RIGHT);
// 	VWStraight(25, 100);
// 	VWWait();
// 	curr = PSDGet(PSD_RIGHT);
// 	if (curr - prev > EPSILON) {
// 		VWTurn(-ADJUSTING_ANGLE, ADJUSTING_SPEED);
// 		VWWait();
// 	} else if (prev - curr > EPSILON) {
// 		VWTurn(ADJUSTING_ANGLE, ADJUSTING_SPEED);
// 		VWWait();
// 	}
// 	VWStraight(25, 100);
// 	VWWait();
// }