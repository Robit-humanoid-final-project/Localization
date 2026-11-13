#include "stadium_info.hpp"

int defstadiumsize(StadiumSize& size)
{
    size.width = 9;
    size.height = 6;

    return 0;
}

void definelandmarks(std::array<Landmarks, 16>& landmarks)
{
    // L
    landmarks[0] = {0, 1, 0.95, StadiumInfo::A/2, StadiumInfo::B/2};
    landmarks[1] = {0, 2, 0.90, StadiumInfo::A/2, -StadiumInfo::B/2};
    landmarks[2] = {0, 3, 0.85, StadiumInfo::A/2 - StadiumInfo::E, StadiumInfo::F/2};
    landmarks[3] = {0, 4, 0.80, StadiumInfo::A/2 - StadiumInfo::E, -StadiumInfo::F/2};
    landmarks[4] = {0, 5, 0.80, StadiumInfo::A/2 - StadiumInfo::G, StadiumInfo::H/2};
    landmarks[5] = {0, 6, 0.80, StadiumInfo::A/2 - StadiumInfo::G, -StadiumInfo::H/2};

    // T
    landmarks[6] = {1, 7, 0.95, 0.0, StadiumInfo::B/2};
    landmarks[7] = {1, 8, 0.90, 0.0, -StadiumInfo::B/2};
    landmarks[8] = {1, 9, 0.85, StadiumInfo::A/2, StadiumInfo::H/2};
    landmarks[9] = {1, 10, 0.80, StadiumInfo::A/2, -StadiumInfo::H/2};

    // X
    landmarks[10] = {2, 11, 0.95, 0.0, StadiumInfo::J/2};
    landmarks[11] = {2, 12, 0.90, 0.0, -StadiumInfo::J/2};

    // G
    landmarks[12] = {3, 13, 0.95, StadiumInfo::A/2, StadiumInfo::D/2};
    landmarks[13] = {3, 14, 0.90, StadiumInfo::A/2, -StadiumInfo::D/2};
    landmarks[14] = {3, 15, 0.85, StadiumInfo::A/2 + StadiumInfo::C, StadiumInfo::D/2};
    landmarks[15] = {3, 16, 0.80, StadiumInfo::A/2 + StadiumInfo::C, -StadiumInfo::D/2};
}
