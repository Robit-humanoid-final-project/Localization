#ifndef LANDMARK_OBSERVATION_HPP
#define LANDMARK_OBSERVATION_HPP

#include <cstdint>

// Landmark 하나의 관측 정보
struct LandmarkObservation
{
    std::uint8_t type = 0; // topic: landmark 종류 
    std::uint8_t detection_id = 0; // topic: 검출 landmark ID

    float confidence = 0.0f; // topic: 검출 신뢰도
    // 로봇 기준 좌표 (m)
    float x = 0.0f; // topic: 로봇 기준 전방 거리
    float y = 0.0f; // topic: 로봇 기준 좌측 거리
};

#endif
