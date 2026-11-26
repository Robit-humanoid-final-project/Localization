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

#include "mainwindow.hpp"
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

static QString makeVisionLogText(const VisionObservation& observation)
{
    QString status;

    switch (observation.status)
    {
        case VisionStatus::OK:
            status = QStringLiteral("OK");
            break;
        case VisionStatus::NO_DETECTION:
            status = QStringLiteral("NO_DETECTION");
            break;
        case VisionStatus::CAMERA_ERROR:
            status = QStringLiteral("CAMERA_ERROR");
            break;
        case VisionStatus::INTERFACE_ERROR:
            status = QStringLiteral("INTERFACE_ERROR");
            break;
        case VisionStatus::POSITION_ERROR:
            status = QStringLiteral("POSITION_ERROR");
            break;
        default:
            status = QStringLiteral("UNKNOWN");
            break;
    }

    QString text = QStringLiteral("LIVE /vision_packages\nStatus: %1\n")
        .arg(status);

    text += QStringLiteral("Landmarks: %1\n")
        .arg(static_cast<int>(observation.detections.size()));

    for (const auto& detection : observation.detections)
    {
        text += QStringLiteral("  type=%1 id=%2 confidence=%3 x=%4 y=%5\n")
            .arg(static_cast<int>(detection.type))
            .arg(static_cast<int>(detection.detection_id))
            .arg(detection.confidence, 0, 'f', 2)
            .arg(detection.x, 0, 'f', 3)
            .arg(detection.y, 0, 'f', 3);
    }

    text += QStringLiteral("Ball: %1\n")
        .arg(static_cast<int>(observation.ball.size()));

    for (const auto& ball : observation.ball)
    {
        text += QStringLiteral("  confidence=%1 x=%2 y=%3\n")
            .arg(ball.confidence, 0, 'f', 2)
            .arg(ball.x, 0, 'f', 3)
            .arg(ball.y, 0, 'f', 3);
    }

    text += QStringLiteral("Obstacles: %1\n")
        .arg(static_cast<int>(observation.obstacles.size()));

    for (const auto& obstacle : observation.obstacles)
    {
        text += QStringLiteral("  id=%1 confidence=%2 x=%3 y=%4\n")
            .arg(static_cast<int>(obstacle.detection_id))
            .arg(obstacle.confidence, 0, 'f', 2)
            .arg(obstacle.x, 0, 'f', 3)
            .arg(obstacle.y, 0, 'f', 3);
    }

    return text;
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow mainWindow;

    if (!mainWindow.load())
    {
        qDebug() << "UI 로드 실패:" << mainWindow.loadError();
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
    init.mode      = initNode["mode"].as<bool>();
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

    mainWindow.initOverlays(
        landmarks,
        mcl.getParticles(),
        estimatedPose
    );

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

    QStringList imuLogLines;
    imuLogLines << QStringLiteral("Waiting for /Imu...");

    QStringList odometryLogLines;
    odometryLogLines << QStringLiteral("Waiting for /ikcoordinate...");

    auto *visionTimer = new QTimer(&app);
    QObject::connect(visionTimer, &QTimer::timeout, &app,
        [visionNode, imuNode, ikCoordNode, worldModelNode, 
         &mainWindow, &mcl, &estimatedPose, &pending_dyaw_rad, &imuLogLines,
         &odometryLogLines, &yaw_offset_set, &yaw_offset, &imu_world_yaw, init_yaw]()
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

                imuLogLines.append(QStringLiteral("[IMU /Imu] yaw=%1 deg, world_yaw=%2 rad")
                            .arg(yaw_deg, 0, 'f', 2)
                            .arg(imu_world_yaw, 0, 'f', 4));

            }

            while (imuLogLines.size() > 20)
            {
                imuLogLines.removeFirst();
            }

            mainWindow.setImuLog(imuLogLines);

            //IkCoordinate에서 받은 원시 이동량 
            std::int64_t step_x_raw = 0;
            std::int64_t step_y_raw = 0;
            if (ikCoordNode->takeIkUpdate(step_x_raw, step_y_raw))
            {
                odometryLogLines.append(
                    QStringLiteral("[IK /ikcoordinate] x=%1, y=%2 (raw)")
                        .arg(step_x_raw)
                        .arg(step_y_raw));

                OdometryMove move{};
                move.dx = static_cast<double>(step_x_raw) / IK_COORD_UNITS_PER_METER;
                move.dy = static_cast<double>(step_y_raw) / IK_COORD_UNITS_PER_METER;
                move.dyaw = 0.0;

                mcl.setYawFromImu(imu_world_yaw);
                mcl.prediction(move);
                estimatedPose = mcl.estimatePose();
                mainWindow.updateRobotPose(estimatedPose);
                mainWindow.updateParticles(mcl.getParticles());
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

            mainWindow.setOdometryLog(odometryLogLines);

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
                mainWindow.updateRobotPose(estimatedPose);
                mainWindow.updateParticles(mcl.getParticles());
                odometryLogLines.append(
                    QStringLiteral("[MCL pose after vision] x=%1 m, y=%2 m, yaw=%3 rad")
                        .arg(estimatedPose.x, 0, 'f', 4)
                        .arg(estimatedPose.y, 0, 'f', 4)
                        .arg(estimatedPose.yaw, 0, 'f', 4));

                while (odometryLogLines.size() > 20)
                {
                    odometryLogLines.removeFirst();
                }
                mainWindow.setOdometryLog(odometryLogLines);
            }
        
            mainWindow.updateVisionObject(liveObservation, estimatedPose);

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

            
            mainWindow.setVisionLog(makeVisionLogText(liveObservation));
        });
    visionTimer->start(50);

    // 화면 출력
    mainWindow.show();

    const int result = app.exec();
    executor.cancel();
    if (rosThread.joinable())
    {
        rosThread.join();
    }
    rclcpp::shutdown();
    return result;
}
