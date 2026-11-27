#include "mcl.hpp"
#include "stadium_info.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

// 생성자
// ========================================
MCL::MCL(
    std::size_t particle_count,
    double sensor_sigma,
    double noise_x,
    double noise_y,
    double noise_yaw,
    double roughening_x,
    double roughening_y,
    double roughening_yaw,
    const MclInitConfig& init
)
    : sensor_sigma_(sensor_sigma),
      noise_x_(noise_x),
      noise_y_(noise_y),
      noise_yaw_(noise_yaw),
      roughening_x_(roughening_x),
      roughening_y_(roughening_y),
      roughening_yaw_(roughening_yaw)
{
    if(init.mode == true)
    {
        initialized_particles = initializeParticlesAround(particle_count, init.x, init.y, init.yaw, init.sigma_x, init.sigma_y, init.sigma_yaw);
    }
    else
    {
        initialized_particles = initializeParticles(particle_count);
    }
}

// initialize된 particle 반환
// ========================================
const std::vector<Particle>& MCL::getParticles() const
{
    return initialized_particles;
}

void MCL::setYawFromImu(double world_yaw)
{
    constexpr double PI = 3.14159265358979323846;
    static std::mt19937 gen(std::random_device{}());
    std::normal_distribution<double> imu_noise(0.0, imu_yaw_sigma_);

    for(auto& particle : initialized_particles)
    {
        particle.yaw = std::remainder(world_yaw + imu_noise(gen), 2.0*PI);
    }
}
// prediction
//=====================================
void MCL::prediction(const OdometryMove& move)
{

    constexpr double PI = 3.14159265358979323846;

    // 난수 생성
    static std::mt19937 generator(std::random_device{}());

    // Gaussian Noise 분포
    // std::normal_distribution: 정규 분포 객체 
    // normal_distribution(평균, 표준편차) 형식임 
    std::normal_distribution<double> noise_x(0.0, noise_x_);
    std::normal_distribution<double> noise_y(0.0, noise_y_);
    std::normal_distribution<double> noise_yaw(0.0, noise_yaw_);

    //initialized_particles 간접 참조
    //========================================
    for(auto& particle : initialized_particles)
    {
        // Particle마다 독립적인 이동 노이즈 생성
        // X 또는 Y 이동이 있을 때만 위치 Noise 추가
        // 회전량이 0이 아닐 때만 Yaw Noise 추가
        // Noise는 정규 분포를 따르는 수 중에 랜덤으로 결정됨
        const bool has_translation = move.dx != 0.0 || move.dy != 0.0;
        const double noisy_dx = move.dx + (has_translation ? noise_x(generator) : 0.0);
        const double noisy_dy = move.dy + (has_translation ? noise_y(generator) : 0.0);
        const double noisy_dyaw = move.dyaw + (move.dyaw != 0.0 ? noise_yaw(generator) : 0.0);

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

    constexpr double PI = 3.14159265358979323846;

    std::vector<Particle> new_particles;
    new_particles.reserve(N); //N개 분량만큼 메모리 미리 확보

    // 누적 가중치 계산
    // 누적하는 이유: 노이즈 값 때문에 잘못된 위치가 가중치가 높게 나올 경우 방지
    // 누적해서 통계적으로 가중치가 높은 particle 선택 
    std::vector<double> cumulative_weights(N);

    cumulative_weights[0] = initialized_particles[0].weight;

    // 이전 가중치에 현재 가중치를 더해서 누적 가중치 만듦 
    // 어짜피 1일텐데 왜?: 각 particle까지 누적된 중간 과정의 값이 중요한 것임
    for (std::size_t i = 1; i < N; ++i)
    {
        cumulative_weights[i] = cumulative_weights[i - 1] + initialized_particles[i].weight;
    }

    // Systematic Resampling 간격 설정
    // 다른 간격 설정 방법도 구현해서 분석 비교 해보기
    const double step = 1.0 / static_cast<double>(N);

    static std::mt19937 generator(std::random_device{}());

    std::uniform_real_distribution<double> distribution(0.0, step);
    std::normal_distribution<double> roughen_x(0.0, roughening_x_);
    std::normal_distribution<double> roughen_y(0.0, roughening_y_);
    std::normal_distribution<double> roughen_yaw(0.0, roughening_yaw_);


    // 0~ step 사이 난수 생성
    const double start = distribution(generator);

    // 3. Particle 선택
    // ========================
    std::size_t index = 0;

    for (std::size_t i = 0; i < N; ++i)
    {
        // 각 particle마다 target 값이 존재하고 누적 가중치로 나눠진 부분에 target 값이 포함되는지 보면 됨
        const double target = start + static_cast<double>(i) * step;

        while (index < N - 1 && target > cumulative_weights[index])
        {
            ++index;
        }

        Particle selected = initialized_particles[index];

        selected.x = std::clamp(selected.x + roughen_x(generator), -StadiumInfo::A / 2.0, StadiumInfo::A / 2.0);
        selected.y = std::clamp(selected.y + roughen_y(generator), -StadiumInfo::B / 2.0, StadiumInfo::B / 2.0);
        selected.yaw += roughen_yaw(generator);
       
        while (selected.yaw >= PI)
        {
            selected.yaw -= 2.0 * PI;
        }
        while (selected.yaw < -PI)
        {
            selected.yaw += 2.0 * PI;
        }

        // 4. Weight 균등 초기화
        selected.weight = step;

        new_particles.push_back(selected);
    }

    // 5. 기존 particle -> 새로운 particle로 교체(particle 개수는 유지됨)
    initialized_particles = std::move(new_particles);
}

RobotPose MCL::estimatePose() const
{
    RobotPose estimated_pose;

    if (initialized_particles.empty())
    {
        return estimated_pose;
    }

    // 가중 평균 계산을 위한 변수
    double weighted_x = 0.0; // 가중치가 반영된 x 좌표 합 
    double weighted_y = 0.0; // 가중치가 반영된 y 좌표 합

    double weighted_sin_yaw = 0.0; // 가중치가 반영된 sin(yaw) 값
    double weighted_cos_yaw = 0.0; // 가중치가 반영된 cos(yaw) 값 
 
    double weight_sum = 0.0; // 전체 가중치 합

    // 가중 평균 계산: 가중치를 새로 계산하는게 아니라 이미 저장되어있는 가중치를 좌표에 곱함 
    for (const auto& particle : initialized_particles)
    {
        weighted_x += particle.weight * particle.x;
        weighted_y += particle.weight * particle.y;

        weighted_sin_yaw += particle.weight * std::sin(particle.yaw);

        weighted_cos_yaw += particle.weight * std::cos(particle.yaw);

        weight_sum += particle.weight;
    }

    // 가중치 합 확인 
    if (weight_sum <= 0.0)
    {
        return estimated_pose;
    }

    // 가중 평균 계산 결과를 실제 RobotPose에 적용 
    // 가중치 전체 합은 1인데 왜 굳이 나눔?: 오류로 1이 아닐 수도 있는 상횡 대비 
    estimated_pose.x = weighted_x / weight_sum;
    estimated_pose.y = weighted_y / weight_sum;

    // Circular Mean으로 Yaw 계산
    estimated_pose.yaw = std::atan2(weighted_sin_yaw, weighted_cos_yaw);

    return estimated_pose;
}

// Calculate Landmark Likelihood
// landmarks 구초제와 LandmarkObservation 구초제 사이 오차 계산
// ========================================
double MCL::calculateLandmarkLikelihood(const Particle& particle, const LandmarkObservation& observation) const
{
    //landmark 16개 가져오기
    std::array<Landmarks, 16> landmarks;
    definelandmarks(landmarks);

    // particle yaw 정보 저장 
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

        // 이론상 거리
        // particle 월드 좌표랑 landmark 월드 좌표 사이 거리 
        const double dx = landmark.x - particle.x;
        const double dy = landmark.y - particle.y;

        // 월드 좌표 -> 로봇 좌표 변환
        // 월드 좌표를 왜 로봇 좌표로?: 월드 좌표계와 로봇 좌표계의 기준 방향이 다르기 때문
        // estimatedPose: 월드 좌표계임 토픽 쏴줄 땐 이걸로 
        // 지금은 가중치 계산에 필요하기 때문에 로봇 좌표계로 변환하는 것
        const double predicted_x = cos_yaw * dx + sin_yaw * dy;

        const double predicted_y = -sin_yaw * dx + cos_yaw * dy;

        // 예측 좌표와 실제 관측 좌표의 오차
        // observation.x / observation.y: vision에서 관측된 실측 거리
        // predicted.x / predicted.y: particle과 landmark 좌표 사이 이론상의 가리 
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
    const double sigma_squared = sensor_sigma_ * sensor_sigma_;

    //해당 Particle이 Vision에서 관측한 Landmark 위치를 얼마나 잘 설명하는지 나타내는 가능도 반환
    return std::exp(-min_distance_squared / (2.0 * sigma_squared));
}


// Calculate Particle Weights
// calculateLandmarkLikehood에서 받아온 값(가능도)으로 각 particle 별로 가중치 계산
// ========================================
void MCL::calculateWeights(const VisionObservation& observation)
{
    // 정상적인 관측 데이터가 아니면 계산하지 않음
    if (observation.status != VisionStatus::OK || observation.detections.empty() || initialized_particles.empty())
    {
        return;
    }

    std::vector<double> log_weights;
    log_weights.reserve(initialized_particles.size()); //reserve: 메모리 미리 할당

    double max_log_weight = -std::numeric_limits<double>::max();

    // 각 Particle의 Log Weight 계산
    for (const auto& particle : initialized_particles)
    {
        // 일반 가중치 계산 X -> 로그 가중치 계산
        // 로그 가중치 장점: 일반 가중치는 계속 돌리다보면 
        // 최종 가중치는 나중에 일반 가중치로 다시 변환함
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

        // 로그 가중치 메모리에 저장 
        log_weights.push_back(log_weight);

        // 지금까지 계산한 로그 가중치 중 최댓값을 저장
        // 왜 최대값: 일반 가중치로 변환할 때 최대 값을 빼고 계산하기 위해 
        // 최대 값을 왜 뺴는가?: 지수 함수의 승이 0보다 작아서 너무 큰 값 방지 가능 
        max_log_weight = std::max(max_log_weight, log_weight);
    }

    // Log Weight를 일반 Weight로 변환
    double weight_sum = 0.0;

    for (std::size_t i = 0; i < initialized_particles.size(); ++i)
    {
        // 로그 가중치 이용해서 일반 가중치 게산 
        initialized_particles[i].weight = std::exp(log_weights[i] - max_log_weight);

        //전체 가중치 합산
        weight_sum += initialized_particles[i].weight;
    }

    // Weight 정규화
    // 가중치 합을 1로 만드는 작업(확률처럼 해석하기 위함)
    if (weight_sum > 0.0)
    {
        for (auto& particle : initialized_particles)
        {
            particle.weight /= weight_sum;
        }
    }
}


