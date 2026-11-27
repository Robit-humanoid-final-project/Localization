#include "particle.hpp"
#include "stadium_info.hpp"

#include <yaml-cpp/yaml.h>
#include <random>
#include <cmath>
#include <algorithm>

constexpr double PI = 3.14159265358979323846;

std::vector<Particle> initializeParticles(std::size_t count)
{
    std::vector<Particle> particles;
    particles.reserve(count); // reserve(): 벡터가 사용할 메모리 공간 미리 할당하는 함수

    // 난수 생성기
    std::random_device rd; // std::random_device: 무작위 숫자 생성해주는 utility
    std::mt19937 gen(rd()); //std::mt19937: 메르센 트위스터 엔진 초기화

    // 경기장 범위 내에서 위치 무작위 생성
    // std::uniform_real_distribution: 지정된 범위에서 균일하게 뿌리기
    std::uniform_real_distribution<double> distX(-StadiumInfo::A / 2.0, StadiumInfo::A / 2.0);

    std::uniform_real_distribution<double> distY(-StadiumInfo::B / 2.0, StadiumInfo::B / 2.0);
    
    // 방향: -PI ~ PI
    // 초기에는 로봇이 바라보고 있는 방향을 모른다고 가정 
    std::uniform_real_distribution<double> distYaw(-PI, PI);

    // 모든 Particle에 동일한 초기 Weight 부여
    // 가중치 전체 합 = 1
    const double initialWeight = 1.0 / count;

    for (std::size_t i = 0; i < count; ++i)
    {
        Particle particle;
        
        //particle 초기값 설정
        particle.x = distX(gen); 
        particle.y = distY(gen);
        particle.yaw = distYaw(gen);
        particle.weight = initialWeight;

        particles.push_back(particle);
    }

    return particles;
}

std::vector<Particle> initializeParticlesAround(std::size_t count, double x, double y, double yaw, double sigma_x, double sigma_y, double sigma_yaw)
{
    std::vector<Particle> particles;
    particles.reserve(count);

    std::mt19937 gen(std::random_device{}());
    // 로봇 시작 좌표 기준으로 정규분포 생성(편차: sigma)
    std::normal_distribution<double> dx(x, sigma_x);
    std::normal_distribution<double> dy(y, sigma_y);
    std::normal_distribution<double> dyaw(yaw, sigma_yaw);

    for (std::size_t i = 0; i < count; ++i)
    {
        Particle p;
        p.x = std::clamp(dx(gen), -StadiumInfo::A / 2.0, StadiumInfo::A / 2.0);
        p.y = std::clamp(dy(gen), -StadiumInfo::B / 2.0, StadiumInfo::B / 2.0);
        p.yaw = std::remainder(dyaw(gen), 2.0 * PI);   // -PI ~ PI로 정규화
        p.weight = 1.0 / count;
        particles.push_back(p);
    }
    return particles;
}
