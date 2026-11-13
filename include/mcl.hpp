
#ifndef MCL_HPP
#define MCL_HPP

#include <vector>

#include "particle.hpp"
#include "robot_state.hpp"
#include "vision_observation.hpp"

class MCL
{
public:
    explicit MCL(std::size_t particle_count);

    // Particle 접근
    const std::vector<Particle>& getParticles() const;

    // Vision Observation을 이용한 Weight 계산
    void calculateWeights(const VisionObservation& observation);

    //가중치 계산
    void prediction(const OdometryMove& move);

    void resample();

    RobotPose estimatePose() const;


private:
    std::vector<Particle> particles_;

    // Landmark 관측값과 예측값의 오차를 이용한 likelihood 계산
    double calculateLandmarkLikelihood(
        const Particle& particle,
        const LandmarkObservation& observation
    ) const;
};

#endif
