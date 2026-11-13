
#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QUiLoader>
#include <QFile>
#include <QLabel>
#include <QPainter>
#include <QColor>
#include <QFileInfo>
#include <QDebug>
#include <QDir>

#include <array>
#include <cmath>
#include <vector>
#include <algorithm>
#include "stadium_info.hpp"
#include "robot_state.hpp"
#include "odometry.hpp"
#include "particle.hpp"
#include "mcl.hpp"
#include "vision_observation.hpp"


class LandmarkOverlay : public QWidget
{
public:
    LandmarkOverlay(QWidget *parent,
                    const std::array<Landmarks, 16>& landmarks)
        : QWidget(parent), landmarks(landmarks)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
    }

protected:
    //UI에 표시
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        float scaleX = width() / (StadiumInfo::A + 2 * StadiumInfo::K);
        float scaleY = height() / (StadiumInfo::B + 2 * StadiumInfo::K);

        for (const auto& landmark : landmarks)
        {
            // 기존 좌상단 원점 좌표를 중앙 원점 좌표로 변환
            float px = (landmark.x + StadiumInfo::A / 2.0f + StadiumInfo::K) * scaleX;
            float py = (StadiumInfo::B / 2.0f + StadiumInfo::K - landmark.y) * scaleY;
        
            
            qDebug() << "Overlay Size:" << width() << height()
                << "ID:" << landmark.detection_id
                << "Pixel:" << px << py;
            
            QColor color;

            // LTXG 색상 설정 
            switch (landmark.type)
            {
                case 0: color = Qt::red; break;
                case 1: color = Qt::blue; break;
                case 2: color = Qt::yellow; break;
                case 3: color = Qt::magenta; break;
                default: color = Qt::white; break;
            }

            painter.setBrush(color);
            painter.setPen(QPen(Qt::black, 1));

            painter.drawEllipse(QPointF(px, py), 5, 5);

            // 텍스트는 화면 좌표계에서 출력
            painter.save();
            painter.resetTransform();


            painter.restore();
        }
    }

    private:
         std::array<Landmarks, 16> landmarks;
    };

class ParticleOverlay : public QWidget
{
public:
    explicit ParticleOverlay(QWidget *parent)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
    }

    // Particle 데이터 전달
    void setParticles(const std::vector<Particle>& particles)
    {
        particles_ = particles;
        update();
    }

protected:
    // UI에 표시
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        const double scaleX = width() / (StadiumInfo::A + 2 * StadiumInfo::K);

        const double scaleY = height() / (StadiumInfo::B + 2 * StadiumInfo::K);

        // Particle 표시 스타일
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(255, 0, 0, 150));

        for (const auto& particle : particles_)
        {
            // 월드 좌표 -> 화면 좌표
            const double px = (particle.x + StadiumInfo::A / 2.0 + StadiumInfo::K) * scaleX;

            const double py = (StadiumInfo::B / 2.0 + StadiumInfo::K - particle.y) * scaleY;

            // Particle 표시
            painter.drawEllipse(QPointF(px, py),2.0,2.0);
        }
    }

private:
    std::vector<Particle> particles_;
};

class RobotOverlay : public QWidget
{
public:
    explicit RobotOverlay(QWidget *parent)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
    }

    // 외부에서 로봇 위치 및 방향 갱신
    void setPose(const RobotPose& pose)
    {
        pose_ = pose;
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // 경기장 좌표를 화면 좌표로 변환
        const double scaleX = width() / (StadiumInfo::A + 2 * StadiumInfo::K);

        const double scaleY = height() / (StadiumInfo::B + 2 * StadiumInfo::K);

        const double px = (pose_.x + StadiumInfo::A / 2.0 + StadiumInfo::K) * scaleX;

        const double py = (StadiumInfo::B / 2.0 + StadiumInfo::K - pose_.y) * scaleY;

        // 로봇 크기
        constexpr double ROBOT_RADIUS = 16.0;

        // 진행 방향 표시 길이
        constexpr double HEADING_LENGTH = 28.0;

        // 로봇 본체
        painter.setBrush(QColor(0, 180, 0));
        painter.setPen(QPen(Qt::black, 2));

        painter.drawEllipse(QPointF(px, py), ROBOT_RADIUS, ROBOT_RADIUS);

        // 진행 방향 계산
        const double headingX = px + HEADING_LENGTH * std::cos(pose_.yaw);

        const double headingY = py - HEADING_LENGTH * std::sin(pose_.yaw);

        // 방향 표시선
        painter.setPen(QPen(Qt::black, 3));

        painter.drawLine(QPointF(px, py), QPointF(headingX, headingY));

        // 로봇 중심점
        painter.setBrush(Qt::white);
        painter.setPen(Qt::NoPen);

        painter.drawEllipse(QPointF(px, py), 3, 3);
    }

private:
    RobotPose pose_;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // UI 파일 로드
    QUiLoader loader;

    QString uiPath = "/home/choiyuns/Desktop/robit/예비/Final Project/Localization/Localization.ui";

    QFile file(uiPath);

    if (!file.open(QFile::ReadOnly))
    {
        qDebug() << "ERROR: UI 파일 열기 실패:" << file.errorString();
        return -1;
    }

    QDir::setCurrent(QFileInfo(uiPath).absolutePath());

    QWidget *window = loader.load(&file);
    file.close();

    if (!window)
    {
        qDebug() << "ERROR: UI 로드 실패:" << loader.errorString();
        return -1;
    }

    // Landmark 데이터 생성
    std::array<Landmarks, 16> landmarks;
    definelandmarks(landmarks);

    // Ground Truth: 실제 로봇의 정답 Pose
    RobotPose groundTruth;

    groundTruth.x = -3.0;
    groundTruth.y = 0.0;
    groundTruth.yaw = 0.0;

    // MCL 객체 생성 및 Particle 초기화
    //particle 2000개 생성
    MCL mcl(2000); 

    // 이동 전 첫 번째 Particle 저장
    Particle before = mcl.getParticles()[0];
    
    // Odometry 이동량 설정
    OdometryMove move;

    move.dx = 1.0;
    move.dy = 0.0;
    move.dyaw = 0.0;

    // 모든 Particle Prediction 실행
    mcl.prediction(move);

    // 이동 후 첫 번째 Particle
    Particle after = mcl.getParticles()[0];

    qDebug() << "========== Prediction Test ==========";

    qDebug() << "Before:";
    qDebug() << "X:" << before.x;
    qDebug() << "Y:" << before.y;
    qDebug() << "Yaw:" << before.yaw;

    qDebug() << "After:";
    qDebug() << "X:" << after.x;
    qDebug() << "Y:" << after.y;
    qDebug() << "Yaw:" << after.yaw;

    VisionObservation observation;
    observation.status = VisionStatus::OK;

    // 관측할 랜드마크 인덱스
    const std::array<int, 4> landmark_indices = {2, 6, 10, 12};

    for (std::size_t i = 0; i < landmark_indices.size(); ++i)
    {
        const auto& landmark = landmarks[landmark_indices[i]];

        const double dx = landmark.x - groundTruth.x;
        const double dy = landmark.y - groundTruth.y;

        const double cos_yaw = std::cos(groundTruth.yaw);
        const double sin_yaw = std::sin(groundTruth.yaw);

        LandmarkObservation detection;

        detection.type = landmark.type;
        detection.detection_id = static_cast<std::uint8_t>(i + 1);
        detection.confidence = 0.95f;

        detection.x = cos_yaw * dx + sin_yaw * dy;
        detection.y = -sin_yaw * dx + cos_yaw * dy;

        observation.detections.push_back(detection);
    }

    // Particle Weight 계산
    mcl.calculateWeights(observation);

    // MCL Pose Estimation
    RobotPose estimated_pose = mcl.estimatePose();

    qDebug() << "\n========== MCL Pose Estimation ==========";
    qDebug() << "Estimated Pose:";
    qDebug() << "X:" << estimated_pose.x;
    qDebug() << "Y:" << estimated_pose.y;
    qDebug() << "Yaw:" << estimated_pose.yaw;

    double position_error = std::sqrt(
        std::pow(estimated_pose.x - groundTruth.x, 2) +
        std::pow(estimated_pose.y - groundTruth.y, 2)
    );

    qDebug() << "Position Error:" << position_error << "m";
    qDebug() << "========================================";

    // Resampling 전
    double weight_sum_before = 0.0;

    for (const auto& particle : mcl.getParticles())
    {
        weight_sum_before += particle.weight;
    }

    qDebug() << "\n========== Resampling Test ==========";
    qDebug() << "Before Resampling";
    qDebug() << "Particle Count:" << mcl.getParticles().size();
    qDebug() << "Weight Sum:" << weight_sum_before;

    // Resampling 실행
    mcl.resample();

    // Resampling 후
    double weight_sum_after = 0.0;
    double min_weight = 1.0;
    double max_weight = 0.0;

    for (const auto& particle : mcl.getParticles())
    {
        weight_sum_after += particle.weight;

        min_weight = std::min(min_weight, particle.weight);
        max_weight = std::max(max_weight, particle.weight);
    }

    qDebug() << "\nAfter Resampling";
    qDebug() << "Particle Count:" << mcl.getParticles().size();
    qDebug() << "Weight Sum:" << weight_sum_after;
    qDebug() << "Min Weight:" << min_weight;
    qDebug() << "Max Weight:" << max_weight;
    qDebug() << "=====================================";

    qDebug() << "Ground Truth:"
            << groundTruth.x
            << groundTruth.y
            << groundTruth.yaw;

    double weightSum = 0.0;

    for (const auto& particle : mcl.getParticles())
    {
        weightSum += particle.weight;
    }

    qDebug() << "================================";
    qDebug() << "Particle Count:" << mcl.getParticles().size();
    qDebug() << "Weight Sum:" << weightSum;
    qDebug() << "================================";

    // ========================================
    // 가장 높은 Weight를 가진 Particle 확인
    // ========================================

    const auto& particles = mcl.getParticles();

    auto bestParticle = std::max_element(
        particles.begin(),
        particles.end(),
        [](const Particle& a, const Particle& b)
        {
            return a.weight < b.weight;
        }
    );

    const double positionError = std::hypot(
        bestParticle->x - groundTruth.x,
        bestParticle->y - groundTruth.y
    );

    qDebug() << "Best Particle:";
    qDebug() << "X:" << bestParticle->x;
    qDebug() << "Y:" << bestParticle->y;
    qDebug() << "Yaw:" << bestParticle->yaw;
    qDebug() << "Weight:" << bestParticle->weight;
    qDebug() << "Position Error:" << positionError << "m";

    // UI 내부 QLabel 검색
    QLabel *stadiumLabel =
        window->findChild<QLabel*>("label");

    if (!stadiumLabel)
    {
        qDebug() << "QLabel을 찾을 수 없습니다.";
        delete window;
        return -1;
    }

    // QLabel 위에 Landmark Overlay 생성
    auto *overlay =
        new LandmarkOverlay(stadiumLabel, landmarks);

    // Particle Overlay 생성
    auto *particleOverlay =
        new ParticleOverlay(stadiumLabel);

    particleOverlay->setGeometry(stadiumLabel->rect());
    particleOverlay->setParticles(mcl.getParticles());
    particleOverlay->show();

    overlay->setGeometry(stadiumLabel->rect());
    overlay->show();

    // Robot Overlay 생성
    auto *robotOverlay = new RobotOverlay(stadiumLabel);

    robotOverlay->setGeometry(stadiumLabel->rect());

    // 초기 로봇 위치 설정
    RobotPose initialPose;

    initialPose.x = -3.0;//Topic: 실제 dx값 
    initialPose.y = 0.0; //Topic: 실제 dy 값 
    initialPose.yaw = 0.0; //Topic: 실제 dyaw값 

    robotOverlay->setPose(groundTruth);
    robotOverlay->show();
    // 화면 출력
    window->show();

    return app.exec();
}
