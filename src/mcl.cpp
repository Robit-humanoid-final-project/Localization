#include "mcl.hpp"
#include "stadium_info.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

// 임시 센서 측정 표준편차 (m)
// 실제 Vision 측정 오차에 따라 추후 조정
constexpr double SENSOR_SIGMA = 0.2;

// 생성자
// ========================================
MCL::MCL(std::size_t particle_count)
{
    initialized_particles = initializeParticles(particle_count);
}

// initialize된 particle 반환
// ========================================
const std::vector<Particle>& MCL::getParticles() const
{
    return initialized_particles;
}

// prediction
//=====================================
void MCL::prediction(const OdometryMove& move)
{
    
    // Motion Noise 표준 편차
    constexpr double NOISE_X = 0.02;
    constexpr double NOISE_Y = 0.02;
    constexpr double NOISE_YAW = 0.01;

    constexpr double PI = 3.14159265358979323846;

    // 난수 생성
    static std::mt19937 generator(std::random_device{}());

    // Gaussian Noise 분포
    // std::normal_distribution: 정규 분포 객체 
    std::normal_distribution<double> noise_x(0.0, NOISE_X);
    std::normal_distribution<double> noise_y(0.0, NOISE_Y);
    std::normal_distribution<double> noise_yaw(0.0, NOISE_YAW);

    //initialized_particles 간접 참조
    //========================================
    for(auto& particle : initialized_particles)
    {
        // Particle마다 독립적인 이동 노이즈 생성
        const bool has_translation = move.dx != 0.0 || move.dy != 0.0;
        const double noisy_dx = move.dx
            + (has_translation ? noise_x(generator) : 0.0);
        const double noisy_dy = move.dy
            + (has_translation ? noise_y(generator) : 0.0);
        const double noisy_dyaw = move.dyaw
            + (move.dyaw != 0.0 ? noise_yaw(generator) : 0.0);

        const double cos_yaw = std::cos(particle.yaw);
        const double sin_yaw = std::sin(particle.yaw);

        // 로봇 좌표계 이동량을 월드 좌표계로 변환
        particle.x += noisy_dx * cos_yaw - noisy_dy * sin_yaw;

        particle.y += noisy_dx * sin_yaw + noisy_dy * cos_yaw;

        // 방향각 업데이트
        particle.yaw += noisy_dyaw;

        // Yaw 정규화: -400, +720같이 -360, 360을 넘어가면 다시 0으로 돌아오도록 설정
        while (particle.yaw >= PI)
        {
            particle.yaw -= 2.0 * PI;
        }

        while (particle.yaw < -PI)
        {
            particle.yaw += 2.0 * PI;
        }
    }
}

// resampling: 가중치 높은 파티클 좌표에 복제
//===========================
void MCL::resample()
{
    if(initialized_particles.empty())
    {
        return;
    }

    const std::size_t N = initialized_particles.size();

    std::vector<Particle> new_particles;
    new_particles.reserve(N); //N개 분량만큼 메모리 미리 확보

    // 1. 누적 가중치 계산
    // 누적하는 이유: 노이즈 값 때문에 잘못된 위치가 가중치가 높게 나올 경우 방지
    // 누적해서 통계적으로 가중치가 높은 particle 선택 
    std::vector<double> cumulative_weights(N);

    cumulative_weights[0] = initialized_particles[0].weight;

    for (std::size_t i = 1; i < N; ++i)
    {
        cumulative_weights[i] = cumulative_weights[i - 1] + initialized_particles[i].weight;
    }

    // 2. Systematic Resampling 간격 설정
    const double step = 1.0 / static_cast<double>(N);

    static std::mt19937 generator(std::random_device{}());

    std::uniform_real_distribution<double> distribution(0.0, step);

    const double start = distribution(generator);

    // 3. Particle 선택
    std::size_t index = 0;

    for (std::size_t i = 0; i < N; ++i)
    {
        const double target = start + static_cast<double>(i) * step;

        while (index < N - 1 && target > cumulative_weights[index])
        {
            ++index;
        }

        Particle selected = initialized_particles[index];

        // 4. Weight 균등 초기화
        selected.weight = step;

        new_particles.push_back(selected);
    }

    // 5. 기존 Particle 집합 교체
    initialized_particles = std::move(new_particles);
}

RobotPose MCL::estimatePose() const
{
    RobotPose estimated_pose;

    if (initialized_particles.empty())
    {
        return estimated_pose;
    }

    double weighted_x = 0.0;
    double weighted_y = 0.0;

    double weighted_sin_yaw = 0.0;
    double weighted_cos_yaw = 0.0;

    double weight_sum = 0.0;

    // 1. Weighted Mean 계산
    for (const auto& particle : initialized_particles)
    {
        weighted_x += particle.weight * particle.x;
        weighted_y += particle.weight * particle.y;

        weighted_sin_yaw += particle.weight * std::sin(particle.yaw);

        weighted_cos_yaw += particle.weight * std::cos(particle.yaw);

        weight_sum += particle.weight;
    }

    // 2. Weight 정규화
    if (weight_sum <= 0.0)
    {
        return estimated_pose;
    }

    estimated_pose.x = weighted_x / weight_sum;
    estimated_pose.y = weighted_y / weight_sum;

    // 3. Circular Mean으로 Yaw 계산
    estimated_pose.yaw = std::atan2(weighted_sin_yaw, weighted_cos_yaw);

    return estimated_pose;
}


// Calculate Landmark Likelihood
// ========================================
double MCL::calculateLandmarkLikelihood(const Particle& particle, const LandmarkObservation& observation) const
{
    std::array<Landmarks, 16> landmarks;
    definelandmarks(landmarks);

    const double cos_yaw = std::cos(particle.yaw);
    const double sin_yaw = std::sin(particle.yaw);

    double min_distance_squared = std::numeric_limits<double>::max();

    // 경기장의 모든 Landmark를 확인
    for (const auto& landmark : landmarks)
    {
        // Landmark 종류가 다르면 제외
        if (landmark.type != observation.type)
        {
            continue;
        }

        // 월드 좌표에서 Particle까지의 상대 위치
        const double dx = landmark.x - particle.x;
        const double dy = landmark.y - particle.y;

        // 월드 좌표 -> 로봇 좌표 변환
        const double predicted_x = cos_yaw * dx + sin_yaw * dy;

        const double predicted_y = -sin_yaw * dx + cos_yaw * dy;

        // 예측 좌표와 실제 관측 좌표의 오차
        const double error_x = predicted_x - observation.x;

        const double error_y = predicted_y - observation.y;

        const double distance_squared = error_x * error_x + error_y * error_y;

        // 가장 가까운 동일 종류 Landmark 선택
        min_distance_squared = std::min(min_distance_squared, distance_squared);
    }

    // 일치하는 종류의 Landmark가 없는 경우
    if (min_distance_squared == std::numeric_limits<double>::max())
    {
        return 1e-12;
    }

    // Gaussian likelihood
    const double sigma_squared = SENSOR_SIGMA * SENSOR_SIGMA;

    return std::exp(-min_distance_squared / (2.0 * sigma_squared));
}


// Calculate Particle Weights
// ========================================

void MCL::calculateWeights(const VisionObservation& observation)
{
    // 정상적인 관측 데이터가 아니면 계산하지 않음
    if (observation.status != VisionStatus::OK || observation.detections.empty() || initialized_particles.empty())
    {
        return;
    }

    std::vector<double> log_weights;
    log_weights.reserve(initialized_particles.size());

    double max_log_weight = -std::numeric_limits<double>::max();

    // 각 Particle의 Log Weight 계산
    for (const auto& particle : initialized_particles)
    {
        double log_weight = 0.0;

        for (const auto& detection : observation.detections)
        {
            const double likelihood = calculateLandmarkLikelihood(particle, detection);

            // Confidence를 likelihood에 반영
            const double confidence = std::clamp(static_cast<double>(detection.confidence), 0.0, 1.0);

            if (confidence > 0.0)
            {
                log_weight += confidence * std::log(std::max(likelihood, 1e-12));
            }
        }

        log_weights.push_back(log_weight);

        max_log_weight = std::max(max_log_weight, log_weight);
    }

    // Log Weight를 일반 Weight로 변환
    double weight_sum = 0.0;

    for (std::size_t i = 0; i < initialized_particles.size(); ++i)
    {
        initialized_particles[i].weight = std::exp(log_weights[i] - max_log_weight);

        weight_sum += initialized_particles[i].weight;
    }

    // Weight 정규화
    if (weight_sum > 0.0)
    {
        for (auto& particle : initialized_particles)
        {
            particle.weight /= weight_sum;
        }
    }
}
