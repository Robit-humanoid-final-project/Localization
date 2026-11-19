#ifndef MCL_HPP
#define MCL_HPP

#include <cstddef>
#include <vector>

#include "particle.hpp"
#include "robot_state.hpp"
#include "vision_observation.hpp"

class MCL
{
public:
    MCL(
        std::size_t particle_count,
        double sensor_sigma,
        double noise_x, 
        double noise_y, 
        double noise_yaw,
        double roughening_x,
        double roughening_y,
        double roughening_yaw
    );

    // Particle 접근
    const std::vector<Particle>& getParticles() const;

    // Vision Observation을 이용한 Weight 계산
    void calculateWeights(const VisionObservation& observation);

    //가중치 계산
    void prediction(const OdometryMove& move);

    void resample();

    RobotPose estimatePose() const;


private:
    //particle 정보 저장하는 vector
    std::vector<Particle> initialized_particles;

    // Landmark 관측값과 예측값의 오차를 이용한 likelihood 계산
    double calculateLandmarkLikelihood(const Particle& particle, const LandmarkObservation& observation) const;

    // MCL 파라미터
    double sensor_sigma_;

    double noise_x_;
    double noise_y_;
    double noise_yaw_;

    double roughening_x_;
    double roughening_y_;
    double roughening_yaw_;
};

#endif
