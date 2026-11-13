
#ifndef VISION_OBSERVATION_HPP
#define VISION_OBSERVATION_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "landmark_observation.hpp"

// Vision 처리 상태
enum class VisionStatus : std::uint8_t
{
    OK = 0,
    NO_DETECTION = 1,
    CAMERA_ERROR = 2,
    INTERFACE_ERROR = 3,
    POSITION_ERROR = 4
};

// Vision Topic 전체 관측 데이터
struct VisionObservation
{
    // Header 정보
    std::int32_t stamp_sec = 0; // topic: 메시지 수신 데이터의 초 단위 시간
    std::uint32_t stamp_nanosec = 0; // topic: 나노초 단위 시간 
    std::string frame_id = "base_footprint"; // topic: 좌표계 정보


    // Vision 처리 상태
    VisionStatus status = VisionStatus::NO_DETECTION; // topic: Vision 처리 상태

    // 해당 프레임에서 검출된 Landmark 목록
    std::vector<LandmarkObservation> detections; // topic: 검출된 landmark 목록 
};

#endif
