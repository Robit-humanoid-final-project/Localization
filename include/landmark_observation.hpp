
#ifndef LANDMARK_OBSERVATION_HPP
#define LANDMARK_OBSERVATION_HPP

#include <cstdint>

// Landmark 하나의 관측 정보
struct LandmarkObservation
{
    std::uint8_t type = 0;
    std::uint8_t detection_id = 0;

    float confidence = 0.0f;

    // 로봇 기준 좌표 (m)
    float x = 0.0f;
    float y = 0.0f;
};

#endif
