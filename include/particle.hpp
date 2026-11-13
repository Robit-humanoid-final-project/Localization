#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>
#include <cstddef>

// 하나의 Particle이 가정하는 로봇의 위치와 방향
struct Particle
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double weight = 0.0;
};

// Particle 초기화 함수
std::vector<Particle> initializeParticles(std::size_t count);

#endif