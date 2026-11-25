#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

//world 좌표계
//==========================
struct RobotPose
{
    double x = 0.0; //월드 좌표 X
    double y = 0.0; //월드 좌표 Y
    double yaw = 0.0; //월드 좌표 Yaw
}; 


//Odometry 이동량
//==========================
struct OdometryMove
{
    double dx = 0.0; // 로봇 기준 전방 odometry 이동량
    double dy = 0.0; // 로봇 기준 좌측 odometry 이동량 
    double dyaw = 0.0; // 방향 변화량 
}; 

#endif
