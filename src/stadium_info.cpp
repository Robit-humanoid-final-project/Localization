#include "stadium_info.hpp"
#include "subscribe/vision_subscriber.hpp"

int defstadiumsize(StadiumSize& size)
{
    size.width = 9;
    size.height = 6;

    return 0;
}

void definelandmarks(std::array<Landmarks, 16>& landmarks)
{
    LandmarkObservation landmark;

    // L
    landmarks[0] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2, StadiumInfo::B/2};
    landmarks[1] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2, -StadiumInfo::B/2};
    landmarks[2] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2 - StadiumInfo::E, StadiumInfo::F/2};
    landmarks[3] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2 - StadiumInfo::E, -StadiumInfo::F/2};
    landmarks[4] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2 - StadiumInfo::G, StadiumInfo::H/2};
    landmarks[5] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2 - StadiumInfo::G, -StadiumInfo::H/2};

    // T
    landmarks[6] = {landmark.type, landmark.detection_id, landmark.confidence, 0.0, StadiumInfo::B/2};
    landmarks[7] = {landmark.type, landmark.detection_id, landmark.confidence, 0.0, -StadiumInfo::B/2};
    landmarks[8] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2, StadiumInfo::H/2};
    landmarks[9] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2, -StadiumInfo::H/2};
    landmarks[10] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2, StadiumInfo::F/2};
    landmarks[11] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2, -StadiumInfo::F/2}; 

    // X
    landmarks[12] = {landmark.type, landmark.detection_id, landmark.confidence, 0.0, StadiumInfo::J/2};
    landmarks[13] = {landmark.type, landmark.detection_id, landmark.confidence, 0.0, -StadiumInfo::J/2};

    // G
    landmarks[14] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2, StadiumInfo::D/2};
    landmarks[15] = {landmark.type, landmark.detection_id, landmark.confidence, StadiumInfo::A/2, -StadiumInfo::D/2};
}
