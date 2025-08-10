#ifndef __ALGORITHM_SOLVE_TRAJECTORY_H
#define __ALGORITHM_SOLVE_TRAJECTORY_H

struct SolveTrajectory
{
	float aim_x;
	float aim_y;
	float aim_z;
	float hero_x;
	float hero_y;
	float hero_z;
	float bullet_speed;
	float Pitch;
	float Yaw;
	int   fire;
};


extern float  monoDirectionalAirResistanceModel(float s, float v, float angle);
extern float  pitchTrajectoryCompensation(float s, float z, float v);
extern void   autoSolveTrajectory(struct  SolveTrajectory *ST);
extern struct SolveTrajectory  Solvetrajectory;


#endif
