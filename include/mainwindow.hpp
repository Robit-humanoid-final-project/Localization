#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QWidget>
#include <QLabel>
#include <QString>
#include <QStringList>
#include <array>
#include <vector>

#include "stadium_info.hpp"
#include "robot_state.hpp"
#include "particle.hpp"
#include "vision_observation.hpp"

// 전방 선언
class LandmarkOverlay;
class ParticleOverlay;
class RobotOverlay;
class VisionObjectOverlay;

class MainWindow
{
public:
    MainWindow();
    ~MainWindow();

    // UI 로드 성공 여부
    bool load();
    void show();

    // Overlay 초기화 (main.cpp에서 MCL 초기화 후 호출)
    void initOverlays(const std::array<Landmarks, 16>& landmarks, const std::vector<Particle>& particles, const RobotPose& initial_pose);

    // 타이머 콜백에서 호출할 갱신 함수들
    void updateRobotPose(const RobotPose& pose);
    void updateParticles(const std::vector<Particle>& particles);
    void updateVisionObject(const VisionObservation& obs, const RobotPose& pose);
    void setOdometryLog(const QStringList& lines);
    void setVisionLog(const QString& text);
    void setImuLog(const QStringList& lines);

    // main.cpp에서 window->show() 용
    QWidget* window() const { return window_; }

    // UI 로드 실패 사유 확인용
    QString loadError() const { return loadError_; }

private:
    QWidget* window_ = nullptr;
    QLabel* stadiumLabel_ = nullptr;

    LandmarkOverlay* landmarkOverlay_ = nullptr;
    ParticleOverlay* particleOverlay_ = nullptr;
    RobotOverlay* robotOverlay_ = nullptr;
    VisionObjectOverlay* visionObjectOverlay_ = nullptr;

    QString loadError_;
};

#endif // MAINWINDOW_HPP