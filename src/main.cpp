
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
#include <QStringList>
#include <QTimer>

#include <array>
#include <cmath>
#include <utility>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "stadium_info.hpp"
#include "robot_state.hpp"
#include "odometry.hpp"
#include "particle.hpp"
#include "mcl.hpp"
#include "vision_observation.hpp"
#include "subscribe/vision_subscriber.hpp"
#include "subscribe/imu_subscriber.hpp"
#include "subscribe/ik_coord_subscriber.hpp"
#include "publish/world_model_publisher.hpp"


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

        // 핑크색 L 타입 랜드마크 표시(디버깅용)
        for (const auto& landmark : observation_.detections)
        {
            // type 0 = L 타입 (핑크색)
            if (landmark.type != 0)
                continue;

            // 로봇 기준 좌표 -> 월드 좌표 -> UI 픽셀 좌표
            const QPointF point = robotToPixel(landmark.x, landmark.y);

            painter.setBrush(QColor(255, 105, 180));
            painter.setPen(QPen(Qt::black, 1));

            // 핑크색 랜드마크 표시
            painter.drawEllipse(point, 8.0, 8.0);

            // Detection ID 표시
            painter.drawText(
                point + QPointF(10.0, -10.0),
                QStringLiteral("L%1").arg(landmark.detection_id));
        }

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

    // localization 패키지의 설정 파일 경로 가져오기
    std::string package_path = ament_index_cpp::get_package_share_directory("localization");

    // YAML 파일 읽기
    std::string config_path = package_path + "/config/localization.yaml"; 
    YAML::Node config = YAML::LoadFile(config_path);

    // Particle 개수 가져오기(yaml에서 particle 개수 읽어옴)
   // Particle 개수
    std::size_t particle_count = config["mcl"]["particle_count"].as<std::size_t>();

    // Sensor 파라미터
    double sensor_sigma = config["mcl"]["sensor_sigma"].as<double>();

    // Prediction 파라미터
    double noise_x = config["mcl"]["prediction"]["noise_x"].as<double>();

    double noise_y = config["mcl"]["prediction"]["noise_y"].as<double>();

    double noise_yaw = config["mcl"]["prediction"]["noise_yaw"].as<double>();

    // Resampling 파라미터
    double roughening_x = config["mcl"]["resampling"]["roughening_x"].as<double>();

    double roughening_y = config["mcl"]["resampling"]["roughening_y"].as<double>();

    double roughening_yaw = config["mcl"]["resampling"]["roughening_yaw"].as<double>();

    MclInitConfig init;
    const auto initNode = config["mcl"]["init"];
    init.mode      = initNode["mode"].as<std::string>();
    init.x         = initNode["x"].as<double>();
    init.y         = initNode["y"].as<double>();
    init.yaw       = initNode["yaw"].as<double>();
    init.sigma_x   = initNode["sigma_x"].as<double>();
    init.sigma_y   = initNode["sigma_y"].as<double>();
    init.sigma_yaw = initNode["sigma_yaw"].as<double>();

    // MCL 초기화
    MCL mcl(
        particle_count,
        sensor_sigma,
        noise_x,
        noise_y,
        noise_yaw,
        roughening_x,
        roughening_y,
        roughening_yaw,
        init
    );

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
    const QString visionDisplay = QStringLiteral("Waiting for /vision_Display...");

    if (auto *visionLabel = window->findChild<QLabel*>("vision"))
    {
        visionLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
        visionLabel->setMargin(12);
        visionLabel->setWordWrap(true);
        visionLabel->setText(visionDisplay);
    }

    const QString visionText = QStringLiteral("Waiting for /vision_packages...");
    if (auto *visionLogLabel = window->findChild<QLabel*>("visionlog"))
    {
        visionLogLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
        visionLogLabel->setMargin(12);
        visionLogLabel->setWordWrap(true);
        visionLogLabel->setText(visionText);
    }

    QStringList odometryLogLines;
    odometryLogLines << QStringLiteral("Waiting for /Imu and /ikcoordinate...");
    if (auto *odometryLogLabel = window->findChild<QLabel*>("odometrylog"))
    {
        odometryLogLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
        odometryLogLabel->setMargin(12);
        odometryLogLabel->setWordWrap(true);
        odometryLogLabel->setText(odometryLogLines.join('\n'));
    }

    //ROS2 Node 
    rclcpp::init(0, nullptr);
    auto visionNode = std::make_shared<VisionSubscriber>();
    auto imuNode = std::make_shared<ImuSubscriber>();
    auto ikCoordNode = std::make_shared<IkCoordSubscriber>();
    auto worldModelNode = std::make_shared<WorldModelPublisher>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(visionNode);
    executor.add_node(imuNode);
    executor.add_node(ikCoordNode);
    executor.add_node(worldModelNode);
    
    std::thread rosThread([&executor, &app]()
    {
        executor.spin();
        QMetaObject::invokeMethod(&app, [&app]() { app.quit(); }, Qt::QueuedConnection);
    });

    
    constexpr double IK_COORD_UNITS_PER_METER = 1000.0;
    double pending_dyaw_rad = 0.0;
    

    bool yaw_offset_set = false;
    double yaw_offset = 0.0;
    double imu_world_yaw = init.yaw;
    const double init_yaw = init.yaw;

    auto *visionTimer = new QTimer(window);
    QObject::connect(visionTimer, &QTimer::timeout, window,
        [visionNode, imuNode, ikCoordNode, worldModelNode, objectOverlay, robotOverlay,
         particleOverlay, &mcl, &estimatedPose, &pending_dyaw_rad,
         &odometryLogLines, window, &yaw_offset_set, &yaw_offset, &imu_world_yaw, init_yaw]()
        {
            double yaw_deg = 0.0;
            double dyaw_rad = 0.0;
            
            if (imuNode->takeImuUpdate(yaw_deg, dyaw_rad))
            {
                const double imu_yaw = yaw_deg * M_PI / 180.0;
                if(!yaw_offset_set)
                {
                    yaw_offset = init_yaw - imu_yaw;
                    yaw_offset_set = true;
                }

                imu_world_yaw = std::remainder(imu_yaw + yaw_offset, 2.0 * M_PI);

                odometryLogLines.append(QStringLiteral("[IMU /Imu] yaw=%1 deg, world_yaw=%2 rad")
                            .arg(yaw_deg, 0, 'f', 2)
                            .arg(imu_world_yaw, 0, 'f', 4));

            }

            std::int64_t step_x_raw = 0;
            std::int64_t step_y_raw = 0;
            if (ikCoordNode->takeUpdate(step_x_raw, step_y_raw))
            {
                odometryLogLines.append(
                    QStringLiteral("[IK /ikcoordinate] x=%1, y=%2 (raw)")
                        .arg(step_x_raw)
                        .arg(step_y_raw));

                OdometryMove move{};
                move.dx = static_cast<double>(step_x_raw) / IK_COORD_UNITS_PER_METER;
                move.dy = static_cast<double>(step_y_raw) / IK_COORD_UNITS_PER_METER;
                move.dyaw = 0.0;

                odometryLogLines.append(
                    QStringLiteral("[MCL input] dx=%1 m, dy=%2 m, dyaw=%3 rad")
                        .arg(move.dx, 0, 'f', 4)
                        .arg(move.dy, 0, 'f', 4)
                        .arg(move.dyaw, 0, 'f', 4));

                mcl.prediction(move);
                mcl.setYawFromImu(imu_world_yaw);
                estimatedPose = mcl.estimatePose();
                robotOverlay->setPose(estimatedPose);
                particleOverlay->setParticles(mcl.getParticles());
                odometryLogLines.append(
                    QStringLiteral("[MCL pose after odometry] x=%1 m, y=%2 m, yaw=%3 rad")
                        .arg(estimatedPose.x, 0, 'f', 4)
                        .arg(estimatedPose.y, 0, 'f', 4)
                        .arg(estimatedPose.yaw, 0, 'f', 4));
            }

            while (odometryLogLines.size() > 20)
            {
                odometryLogLines.removeFirst();
            }
            if (auto *odometryLogLabel = window->findChild<QLabel*>("odometrylog"))
            {
                odometryLogLabel->setText(odometryLogLines.join('\n'));
            }

            VisionObservation liveObservation;
            if (!visionNode->getLatestObservation(liveObservation))
            {
                return;
            }

            VisionObservation newObservation;
            if (visionNode->takeNewObservation(newObservation)
                && newObservation.status == VisionStatus::OK
                && !newObservation.detections.empty())
            {
                mcl.calculateWeights(newObservation);
                mcl.resample();
                mcl.setYawFromImu(imu_world_yaw);
                estimatedPose = mcl.estimatePose();
                robotOverlay->setPose(estimatedPose);
                particleOverlay->setParticles(mcl.getParticles());
                odometryLogLines.append(
                    QStringLiteral("[MCL pose after vision] x=%1 m, y=%2 m, yaw=%3 rad")
                        .arg(estimatedPose.x, 0, 'f', 4)
                        .arg(estimatedPose.y, 0, 'f', 4)
                        .arg(estimatedPose.yaw, 0, 'f', 4));

                while (odometryLogLines.size() > 20)
                {
                    odometryLogLines.removeFirst();
                }
                if (auto *odometryLogLabel = window->findChild<QLabel*>("odometrylog"))
                {
                    odometryLogLabel->setText(odometryLogLines.join('\n'));
                }
            }
        
            objectOverlay->setObservation(liveObservation, estimatedPose);

            // WorldModel 발행 데이터 구성
            bool ball_detected = false;
            double ball_world_x = 0.0;
            double ball_world_y = 0.0;

            std::vector<std::pair<double, double>> obstacle_positions;

            const double cos_yaw = std::cos(estimatedPose.yaw);
            const double sin_yaw = std::sin(estimatedPose.yaw);

            // 공의 로봇 기준 좌표를 월드 좌표로 변환
            if (liveObservation.status == VisionStatus::OK && !liveObservation.ball.empty())
            {
                const auto& ball = liveObservation.ball.front();

                ball_world_x = estimatedPose.x + ball.x * cos_yaw - ball.y * sin_yaw;

                ball_world_y = estimatedPose.y + ball.x * sin_yaw + ball.y * cos_yaw;

                ball_detected = true;
            }

            // 장애물의 로봇 기준 좌표를 월드 좌표로 변환
            if (liveObservation.status == VisionStatus::OK)
            {
                for (const auto& obstacle : liveObservation.obstacles)
                {
                    const double world_x = estimatedPose.x + obstacle.x * cos_yaw - obstacle.y * sin_yaw;

                    const double world_y = estimatedPose.y + obstacle.x * sin_yaw + obstacle.y * cos_yaw;

                    obstacle_positions.emplace_back(world_x, world_y);
                }
            }

            // WorldModel 발행
            worldModelNode->publishWorldModel(
                estimatedPose.x,
                estimatedPose.y,
                estimatedPose.yaw,
                ball_detected,
                ball_world_x,
                ball_world_y,
                obstacle_positions
            );
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
