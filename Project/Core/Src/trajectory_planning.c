#include "trajectory_planning.h"
#include <math.h>

// Function to generate cubic spline interpolation for a 5-DOF robot
void CubicSpline(float* start_pos, float* end_pos, float* control_pos1, float* control_pos2, TrajectoryPoint* trajectory, int num_points) {
    for (int j = 0; j < num_points; j++) {
        float t = (float)j / (num_points - 1); // Normalized time
        float t2 = t * t;
        float t3 = t2 * t;

        for (int i = 0; i < 3; i++) { // Loop for x, y, z coordinates
            // Cubic Bézier curve formula
            trajectory[j].position[i] = (1 - t) * (1 - t) * (1 - t) * start_pos[i] +
                                        3 * (1 - t) * (1 - t) * t * control_pos1[i] +
                                        3 * (1 - t) * t * t * control_pos2[i] +
                                        t3 * end_pos[i];
        }
    }
}

void GenerateControlPoints(float* start_pos, float* end_pos, float* control_pos1, float* control_pos2, float offset_factor) {
    float direction[3];
    float length = 0.0f;

    // Calculate the direction vector and its length
    for (int i = 0; i < 3; i++) {
        direction[i] = end_pos[i] - start_pos[i];
        length += direction[i] * direction[i];
    }

    length = sqrtf(length);

    // Normalize the direction vector
    if (length > 0) {
        for (int i = 0; i < 3; i++) {
            direction[i] /= length;
        }
    }

    // Generate control points by offsetting the start and end positions
    for (int i = 0; i < 3; i++) {
        control_pos1[i] = start_pos[i] + direction[i] * offset_factor * length;
        control_pos2[i] = end_pos[i] - direction[i] * offset_factor * length;
    }
}


// Function to generate a trajectory with cubic spline for a 5-DOF robot
void GenerateCurvedTrajectory5DOF(float* start_pos, float* end_pos, float* start_ori, float* end_ori, TrajectoryPoint* trajectory, int num_points) {
    float control_pos1[3], control_pos2[3];
    float offset_factor = 0.5f;

    // Automatically generate control points
    GenerateControlPoints(start_pos, end_pos, control_pos1, control_pos2, offset_factor);

    // Generate cubic spline for position
    CubicSpline(start_pos, end_pos, control_pos1, control_pos2, trajectory, num_points);

    // Interpolate orientation for roll and pitch only
    for (int j = 0; j < num_points; j++) {
        float t = (float)j / (float)(num_points - 1);
        InterpolateOrientation(start_ori, end_ori, t, trajectory[j].orientation);
    }
}

// Function to generate a linear trajectory for a 5-DOF robot
void GenerateTrajectory5DOF(float* start_pos, float* end_pos, float* start_ori, float* end_ori, TrajectoryPoint* trajectory, int num_points) {
    // Generate linear interpolation for position
    for (int i = 0; i < 3; i++) { // Loop for x, y, z coordinates
        float delta = (end_pos[i] - start_pos[i]) / (num_points - 1); // Calculate the increment per step
        for (int j = 0; j < num_points; j++) {
            trajectory[j].position[i] = start_pos[i] + delta * j; // Generate trajectory points linearly
        }
    }

    // Interpolate orientation for roll and pitch only
    for (int j = 0; j < num_points; j++) {
        float t = (float)j / (num_points - 1); // Normalized time
        InterpolateOrientation(start_ori, end_ori, t, trajectory[j].orientation);
    }
}

// Linear interpolation for orientation (roll and pitch only)
void InterpolateOrientation(float* start_ori, float* end_ori, float t, float* result_ori) {
    //for (int i = 0; i < 2; i++) {
        result_ori[0] = start_ori[0] * (1 - t) + end_ori[0] * t; // Linear interpolation
        result_ori[1] = 0.0;
        result_ori[2] = start_ori[2] * (1 - t) + end_ori[2] * t; // Linear interpolation

    //}
}



