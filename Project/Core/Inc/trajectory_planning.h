#ifndef INC_TRAJECTORY_PLANNING_H_
#define INC_TRAJECTORY_PLANNING_H_
#include "stm32f1xx_hal.h"

#define NUM_JOINTS 5      // Number of robot joints
#define TIME_STEP 0.01f   // Time step for the control loop (in seconds)
#define TOTAL_TIME 2.0f   // Total duration of the movement (in seconds)

// Structure to store Cartesian positions and orientations at each time step
typedef struct {
    float position[3];    // [x, y, z] Cartesian coordinates
    float orientation[3]; // [roll, pitch] Euler angles (only two angles for 5-DOF robot)
} TrajectoryPoint;

// Function prototypes
void GenerateTrajectory5DOF(float* start_pos, float* end_pos, float* start_ori, float* end_ori, TrajectoryPoint* trajectory, int num_points);
void GenerateControlPoints(float* start_pos, float* end_pos, float* control_pos1, float* control_pos2, float offset_factor);
void CubicSpline(float* start_pos, float* end_pos, float* control_pos1, float* control_pos2, TrajectoryPoint* trajectory, int num_points);
void GenerateCurvedTrajectory5DOF(float* start_pos, float* end_pos, float* start_ori, float* end_ori, TrajectoryPoint* trajectory, int num_points);
void InterpolateOrientation(float* start_ori, float* end_ori, float t, float* result_ori);


#endif /* INC_TRAJECTORY_PLANNING_H_ */
