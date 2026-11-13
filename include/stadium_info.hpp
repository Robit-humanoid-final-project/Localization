#ifndef STADIUM_INFO_HPP
#define STADIUM_INFO_HPP

#include <array>

struct StadiumSize
{
    int width;
    int height;
};

struct Landmarks
{
    int type;
    int detection_id;
    float confidence;
    float x;
    float y;
};

struct StadiumInfo
{
    static constexpr double A       = 9.0;
    static constexpr double B       = 6.0;
    static constexpr double C       = 0.5;
    static constexpr double D       = 1.8;
    static constexpr double E       = 1.0;
    static constexpr double F       = 3.0;
    static constexpr double G       = 2.0;
    static constexpr double H       = 4.0;
    static constexpr double I_DIST = 1.5;
    static constexpr double J       = 1.5;
    static constexpr double K       = 1.0;
    static constexpr double L_ARC   = 0.0;
    static constexpr double LINE_W  = 0.05;
    static constexpr double MARK_SZ = 0.10;
};

// 함수 선언
int defstadiumsize(StadiumSize& size);

void definelandmarks(std::array<Landmarks, 16>& landmarks);

#endif
