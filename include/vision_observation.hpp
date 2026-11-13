
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
    std::int32_t stamp_sec = 0;
    std::uint32_t stamp_nanosec = 0;
    std::string frame_id = "base_footprint";

    // Vision 처리 상태
    VisionStatus status = VisionStatus::NO_DETECTION;

    // 해당 프레임에서 검출된 Landmark 목록
    std::vector<LandmarkObservation> detections;
};

#endif
