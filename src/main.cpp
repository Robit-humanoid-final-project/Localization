
#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QUiLoader>
#include <QFile>
#include <QLabel>
#include <QPainter>
#include <QPointF>
#include <QColor>
#include <QFileInfo>
#include <QDebug>
#include <QDir>
#include <QString>
#include <QTimer>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "stadium_info.hpp"
#include "robot_state.hpp"
#include "odometry.hpp"
#include "particle.hpp"
#include "mcl.hpp"
#include "vision_observation.hpp"
#include "subscribe/vision_subscriber.hpp"
#include "subscribe/imu_subscriber.hpp"
#include "subscribe/ik_coord_subscriber.hpp"


class LandmarkOverlay : public QWidget
{
public:
    LandmarkOverlay(QWidget *parent, const std::array<Landmarks, 16>& landmarks)
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
                case 0: color = Qt::red; break; // L
                case 1: color = Qt::blue; break; // T
                case 2: color = Qt::yellow; break; // X
                case 3: color = Qt::green; break; //G
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

        // 화면에서 확인한 방향에 맞춰 가로·세로를 교환한 크기
        constexpr double ROBOT_WIDTH_PX = 17.0;
        constexpr double ROBOT_HEIGHT_PX = 25.0;

        // 진행 방향 표시 길이
        constexpr double HEADING_LENGTH = 28.0;

        // 로봇 본체
        painter.setBrush(QColor(0, 180, 0));
        painter.setPen(QPen(Qt::black, 2));

        painter.save();
        painter.translate(px, py);
        constexpr double PI = 3.14159265358979323846;
        painter.rotate(-pose_.yaw * 180.0 / PI);
        painter.drawRect(QRectF(
            -ROBOT_WIDTH_PX / 2.0,
            -ROBOT_HEIGHT_PX / 2.0,
            ROBOT_WIDTH_PX,
            ROBOT_HEIGHT_PX));
        painter.restore();

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

// Draws robot-relative Vision object positions on the field map. When real
// Vision messages arrive, pass their values and the robot pose at that frame.
class VisionObjectOverlay : public QWidget
{
public:
    explicit VisionObjectOverlay(QWidget *parent) : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
    }

    void setObservation(const VisionObservation& observation, const RobotPose& robot_pose)
    {
        observation_ = observation;
        robot_pose_ = robot_pose;
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        if (observation_.status != VisionStatus::OK)
        {
            return;
        }

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // 공은 목록에 값이 있을 때만 표시한다. (현재 토픽 정의상 최대 1개)
        for (const auto& ball : observation_.ball)
        {
            const QPointF point = robotToPixel(ball.x, ball.y);

            painter.setBrush(QColor(255, 140, 0));
            painter.setPen(QPen(Qt::black, 1));
            painter.drawEllipse(point, 14.0, 14.0);
            painter.drawText(point + QPointF(11.0, -8.0), QStringLiteral("Ball"));
        }

        // Cones are rendered as position markers. Their common physical size
        // can be added later if the measured cone radius is needed by the UI.
        painter.setBrush(QColor(125, 70, 20));
        painter.setPen(QPen(Qt::black, 1));

        for (const auto& obstacle : observation_.obstacles)
        {
            const QPointF point = robotToPixel(obstacle.x, obstacle.y);
            const QPointF ractangle[4] = {
                point + QPointF(0.0, -9.0),
                point + QPointF(-8.0, 7.0),
                point + QPointF(8.0, 7.0), 
                point + QPointF(8.0, 7.0)
            };

            painter.drawRect(QRectF(point.x() - 8.0, point.y() - 8.0, 16.0, 16.0));
            painter.drawText(point + QPointF(10.0, -7.0), QStringLiteral("O%1").arg(obstacle.detection_id));
        }
    }

private:
    QPointF robotToPixel(double relative_x, double relative_y) const
    {
        // Convert base_footprint coordinates (forward x, left y) to world.
        const double cos_yaw = std::cos(robot_pose_.yaw);
        const double sin_yaw = std::sin(robot_pose_.yaw);

        const double world_x = robot_pose_.x + relative_x * cos_yaw - relative_y * sin_yaw;
        const double world_y = robot_pose_.y + relative_x * sin_yaw + relative_y * cos_yaw;

        const double scale_x = width() / (StadiumInfo::A + 2.0 * StadiumInfo::K);
        const double scale_y = height() / (StadiumInfo::B + 2.0 * StadiumInfo::K);

        return QPointF((world_x + StadiumInfo::A / 2.0 + StadiumInfo::K) * scale_x, (StadiumInfo::B / 2.0 + StadiumInfo::K - world_y) * scale_y);
    }

    VisionObservation observation_;
    RobotPose robot_pose_;
};

static QString makeVisionDisplayText(const VisionObservation& observation)
{
    QString status;
    switch (observation.status)
    {
        case VisionStatus::OK: status = QStringLiteral("OK"); break;
        case VisionStatus::NO_DETECTION: status = QStringLiteral("NO_DETECTION"); break;
        case VisionStatus::CAMERA_ERROR: status = QStringLiteral("CAMERA_ERROR"); break;
        case VisionStatus::INTERFACE_ERROR: status = QStringLiteral("INTERFACE_ERROR"); break;
        case VisionStatus::POSITION_ERROR: status = QStringLiteral("POSITION_ERROR"); break;
        default: status = QStringLiteral("UNKNOWN"); break;
    }

    QString text = QStringLiteral("Vision Data\n"
                                  "Status: %1 (%2)\n"
                                  "Frame: %3\n"
                                  "Stamp: %4.%5\n\n")
        .arg(status)
        .arg(static_cast<unsigned int>(observation.status))
        .arg(QString::fromStdString(observation.frame_id))
        .arg(observation.stamp_sec)
        .arg(observation.stamp_nanosec, 9, 10, QLatin1Char('0'));

    text += QStringLiteral("Landmarks: %1\n").arg(observation.detections.size());
    for (const auto& detection : observation.detections)
    {
        QString type;
        switch (detection.type)
        {
            case 0: type = QStringLiteral("L"); break;
            case 1: type = QStringLiteral("T"); break;
            case 2: type = QStringLiteral("X"); break;
            case 3: type = QStringLiteral("G"); break;
            default: type = QStringLiteral("?"); break;
        }
        text += QStringLiteral("  %1 | ID %2 | confidence %3 | x %4 m | y %5 m\n")
            .arg(type)
            .arg(detection.detection_id)
            .arg(detection.confidence, 0, 'f', 2)
            .arg(detection.x, 0, 'f', 2)
            .arg(detection.y, 0, 'f', 2);
    }
    text += QLatin1Char('\n');

    if (observation.ball.empty())
    {
        text += QStringLiteral("Ball: not detected\n\n");
    }
    else
    {
        const auto& ball = observation.ball.front();
        text += QStringLiteral("Ball: detected\n"
                               "  confidence: %1\n"
                               "  robot x: %2 m\n"
                               "  robot y: %3 m\n\n")
            .arg(ball.confidence, 0, 'f', 2)
            .arg(ball.x, 0, 'f', 2)
            .arg(ball.y, 0, 'f', 2);
    }

    text += QStringLiteral("Obstacles: %1\n").arg(observation.obstacles.size());
    for (const auto& obstacle : observation.obstacles)
    {
        text += QStringLiteral("  ID %1 | confidence %2 | x %3 m | y %4 m\n")
            .arg(obstacle.detection_id)
            .arg(obstacle.confidence, 0, 'f', 2)
            .arg(obstacle.x, 0, 'f', 2)
            .arg(obstacle.y, 0, 'f', 2);
    }

    return text;
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // UI 파일 로드
    QUiLoader loader;

    const auto shareDirectory = ament_index_cpp::get_package_share_directory("localization");
    const QString uiPath = QString::fromStdString(shareDirectory + "/Localization.ui");

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

    std::array<Landmarks, 16> landmarks;
    definelandmarks(landmarks);

    // Initialize the live MCL estimate from a uniform particle distribution.
    MCL mcl(2000);
    RobotPose estimatedPose = mcl.estimatePose();

    // UI 내부 QLabel 검색
    QLabel *stadiumLabel = window->findChild<QLabel*>("label");

    if (!stadiumLabel)
    {
        qDebug() << "QLabel을 찾을 수 없습니다.";
        delete window;
        return -1;
    }

    // QLabel 위에 Landmark Overlay 생성
    auto *overlay = new LandmarkOverlay(stadiumLabel, landmarks);

    // Particle Overlay 생성
    auto *particleOverlay = new ParticleOverlay(stadiumLabel);

    particleOverlay->setGeometry(stadiumLabel->rect());
    particleOverlay->setParticles(mcl.getParticles());
    particleOverlay->show();

    overlay->setGeometry(stadiumLabel->rect());
    overlay->show();

    // Robot Overlay 생성
    auto *robotOverlay = new RobotOverlay(stadiumLabel);

    robotOverlay->setGeometry(stadiumLabel->rect());

    robotOverlay->setPose(estimatedPose);
    robotOverlay->show();

    // Ball and obstacle positions are robot-relative in the topic. Use the
    // matching robot pose to place them on the world-coordinate field map.
    auto *objectOverlay = new VisionObjectOverlay(stadiumLabel);
    objectOverlay->setGeometry(stadiumLabel->rect());
    VisionObservation waitingObservation;
    objectOverlay->setObservation(waitingObservation, estimatedPose);
    objectOverlay->show();

    // Show the temporary topic values in the existing Vision UI panels.
    const QString visionText = QStringLiteral("Waiting for /vision_packages...");

    if (auto *visionLabel = window->findChild<QLabel*>("vision"))
    {
        visionLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
        visionLabel->setMargin(12);
        visionLabel->setWordWrap(true);
        visionLabel->setText(visionText);
    }

    if (auto *visionLogLabel = window->findChild<QLabel*>("visionlog"))
    {
        visionLogLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
        visionLogLabel->setMargin(12);
        visionLogLabel->setWordWrap(true);
        visionLogLabel->setText(visionText);
    }

    // ROS callbacks run on their own executor thread; UI updates stay on Qt's thread.
    rclcpp::init(0, nullptr);
    auto visionNode = std::make_shared<VisionSubscriber>();
    auto imuNode = std::make_shared<ImuSubscriber>();
    auto ikCoordNode = std::make_shared<IkCoordSubscriber>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(visionNode);
    executor.add_node(imuNode);
    executor.add_node(ikCoordNode);
    
    std::thread rosThread([&executor]() { executor.spin(); });

    // IkCoord values are treated as millimeters based on ik_walk's stride parameters.
    constexpr double IK_COORD_UNITS_PER_METER = 1000.0;
    double pending_dyaw_rad = 0.0;

    auto *visionTimer = new QTimer(window);
    QObject::connect(visionTimer, &QTimer::timeout, window,
        [visionNode, imuNode, ikCoordNode, objectOverlay, robotOverlay,
         particleOverlay, &mcl, &estimatedPose, &pending_dyaw_rad, window]()
        {
            double yaw_deg = 0.0;
            double dyaw_rad = 0.0;
            
            if (imuNode->takeImuUpdate(yaw_deg, dyaw_rad))
            {
                pending_dyaw_rad += dyaw_rad;
                qDebug() << "IMU yaw:" << yaw_deg
                         << "deg, dyaw:" << dyaw_rad << "rad";
            }

            std::int64_t step_x_raw = 0;
            std::int64_t step_y_raw = 0;
            if (ikCoordNode->takeUpdate(step_x_raw, step_y_raw))
            {
                OdometryMove move{};
                move.dx = static_cast<double>(step_x_raw) / IK_COORD_UNITS_PER_METER;
                move.dy = static_cast<double>(step_y_raw) / IK_COORD_UNITS_PER_METER;
                move.dyaw = pending_dyaw_rad;
                pending_dyaw_rad = 0.0;

                mcl.prediction(move);
                estimatedPose = mcl.estimatePose();
                robotOverlay->setPose(estimatedPose);
                particleOverlay->setParticles(mcl.getParticles());

                qDebug() << "OdometryMove (m, rad):"
                         << "dx=" << move.dx
                         << "dy=" << move.dy
                         << "dyaw=" << move.dyaw
                         << "| estimate (m, rad):"
                         << estimatedPose.x << estimatedPose.y << estimatedPose.yaw;
            }

            VisionObservation liveObservation;
            if (!visionNode->getLatestObservation(liveObservation))
            {
                return;
            }

            // Correct the particle distribution once per newly received
            // landmark frame, then publish the corrected pose to the UI.
            VisionObservation newObservation;
            if (visionNode->takeNewObservation(newObservation)
                && newObservation.status == VisionStatus::OK
                && !newObservation.detections.empty())
            {
                mcl.calculateWeights(newObservation);
                mcl.resample();
                estimatedPose = mcl.estimatePose();
                robotOverlay->setPose(estimatedPose);
                particleOverlay->setParticles(mcl.getParticles());

                qDebug() << "MCL vision correction:" << newObservation.detections.size()
                         << "landmarks, estimate (m, rad):"
                         << estimatedPose.x << estimatedPose.y << estimatedPose.yaw;
            }
        
            objectOverlay->setObservation(liveObservation, estimatedPose);
            const QString liveText = makeVisionDisplayText(liveObservation);

            if (auto *label = window->findChild<QLabel*>("vision"))
            {
                label->setText(liveText);
            }
            if (auto *logLabel = window->findChild<QLabel*>("visionlog"))
            {
                logLabel->setText(QStringLiteral("LIVE /vision_packages\n\n") + liveText);
            }
        });
    visionTimer->start(50);

    // 화면 출력
    window->show();

    const int result = app.exec();
    executor.cancel();
    if (rosThread.joinable())
    {
        rosThread.join();
    }
    rclcpp::shutdown();
    return result;
}
