#ifndef VISION_OBSERVATION_HPP
#define VISION_OBSERVATION_HPP

#include <cstdint>
#include <string>
#include <vector>

// Vision이 검출한 랜드마크의 로봇 기준 좌표
struct LandmarkObservation
{
    std::uint8_t type = 0; // topic: landmark 종류 
    std::uint8_t detection_id = 0; // topic: 검출 landmark ID
    float confidence = 0.0; // topic: 검출 신뢰도
    float x = 0.0; // topic: 로봇 기준 전방 거리
    float y = 0.0; // topic: 로봇 기준 좌측 거리
};

//Ball
struct BallObservation
{
    float confidence = 0.0; // topic: ball[].confidence
    float x = 0.0; // topic: ball[].x
    float y = 0.0; // topic: ball[].y
};

//Obstacle
struct ObstacleObservation
{
    std::uint8_t detection_id = 0; // topic: obstacle[].detection_id
    float confidence = 0.0; // topic: obstacle[].confidence
    float x = 0.0; // topic: obstacle[].x
    float y = 0.0; // topic: obstacle[].y
};

// Status
enum class VisionStatus : std::uint8_t
{
    OK = 0,
    NO_DETECTION = 1,
    CAMERA_ERROR = 2,
    INTERFACE_ERROR = 3,
    POSITION_ERROR = 4
};

// 하나의 Vision 프레임 데이터. header와 status를 랜드마크·공·장애물이 공유한다.
struct VisionObservation
{
    // Header 정보
    std::int32_t stamp_sec = 0; // topic: 메시지 수신 데이터의 초 단위 시간
    std::uint32_t stamp_nanosec = 0; // topic: 나노초 단위 시간 
    std::string frame_id = "base_footprint"; // topic: 좌표계 정보


    // 프레임 전체 처리 상태:
    // OK는 위치가 유효한 검출이 하나 이상 있을 때,
    // NO_DETECTION은 정상 처리됐지만 검출이 없을 때 사용한다.
    // 일부 검출만 위치 계산에 성공하면 성공한 항목만 담고 OK로 설정한다.
    VisionStatus status = VisionStatus::NO_DETECTION; // topic: Vision 처리 상태

    // MCL이 사용하는 L/T/X/G 랜드마크 검출 목록
    std::vector<LandmarkObservation> detections; // topic: 검출된 landmark 목록 

    // ball[]: 공 미검출이면 비어 있고, 검출되면 최대 1개 항목을 담는다.
    std::vector<BallObservation> ball;

    // 화면 표시와 향후 장애물 처리를 위한 꼬깔 검출 목록
    std::vector<ObstacleObservation> obstacles;
};

#endif
