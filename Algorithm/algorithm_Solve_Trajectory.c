#include <math.h>
#include <stdio.h>
#include "algorithm_Solve_Trajectory.h"

#define PI      3.1415926535f
#define GRAVITY 9.7907f




float C   = 0.47f;          //空气阻力系数
float rho = 1.225f;         //空气密度
float A   = PI * 21.25f * 0.001f * 21.25f * 0.001f; //大弹丸横截面积
float m   = 44.5f * 0.001f; //大弹丸质量
float t   = 0.5f;           //飞行时间

struct SolveTrajectory  Solvetrajectory;

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    float k = (float)(0.5f * C * rho * A / m);
    //t为给定v与angle时的飞行时间
    t = (float)((exp(k * s) - 1) / (k * v * cos(angle)));
    if(t < 0)
    {
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        //printf("[WRAN]: Exceeding the maximum range!\n");
        //重置t，防止下次调用会出现nan
        t =    0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    //printf("model %f %f\n", t, z);
    return z;
}


/*
@brief pitch轴解算
@param s:m 距离
@param z:m 目标装甲板与地面之间的高度
@param v:m/s
@return angle_pitch:rad  pitch轴从水平面开始的角度，向上为正
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3f*(z - z_actual);
        z_temp = z_temp + dz;
        //printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
        //i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
        if (fabsf(dz) < 0.00001f)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param aim_x:m  被击打装甲板x坐标
@param aim_y:m  被击打装甲板y坐标
@param aim_z:m  被击打装甲板z坐标
@param hero_x:m  我方英雄机x坐标
@param hero_y:m  我方英雄机y坐标
@param hero_z:m  我方英雄机z坐标
@param bullet_speed:m/s  子弹初速度
@reutrn pitch:rad  pitch轴从水平面开始的角度，向上为正
@return yaw:rad  瞄准后英雄在地图中的朝向，顺时针为正,-180度～180度
*/
/*
       x^
        |
        |
   y<---|--   原点为红方补给区
        |
*/
void autoSolveTrajectory(struct  SolveTrajectory *ST)
{
    float s;
    float dz;
		s  = sqrt(ST->aim_x * ST->aim_x + ST->aim_y *ST->aim_y);
    dz = ST->aim_z - ST->hero_z;
    ST->Pitch = pitchTrajectoryCompensation(s,dz,ST->bullet_speed)         * 180.0f / PI; //angle
    ST->Yaw   = atan2(( (ST->aim_y - ST->hero_y)),(ST->aim_x - ST->hero_x))* 180.0f / PI; //angle

}




