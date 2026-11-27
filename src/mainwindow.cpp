#include "mainwindow.hpp"

#include <QUiLoader>
#include <QFile>
#include <QPainter>
#include <QPointF>
#include <QColor>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QPointF>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <utility>

QString makeVisionLogText(const VisionObservation& observation)
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

// Overlay 위젯 정의
// ============================================================

class LandmarkOverlay : public QWidget
{
public:
    LandmarkOverlay(QWidget* parent, const std::array<Landmarks, 16>& landmarks)
        : QWidget(parent), landmarks_(landmarks)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        float scaleX = width() / (StadiumInfo::A + 2 * StadiumInfo::K);
        float scaleY = height() / (StadiumInfo::B + 2 * StadiumInfo::K);

        for (const auto& landmark : landmarks_)
        {
            // 기존 좌상단 원점 좌표를 중앙 원점 좌표로 변환
            float px = (landmark.x + StadiumInfo::A / 2.0f + StadiumInfo::K) * scaleX;
            float py = (StadiumInfo::B / 2.0f + StadiumInfo::K - landmark.y) * scaleY;

            QColor color;

            // LTXG 색상 설정
            switch (landmark.type)
            {
                case 0: color = Qt::red; break;    // L
                case 1: color = Qt::blue; break;   // T
                case 2: color = Qt::yellow; break; // X
                case 3: color = Qt::green; break;  // G
                default: color = Qt::white; break;
            }

            painter.setBrush(color);
            painter.setPen(QPen(Qt::black, 1));
            painter.drawEllipse(QPointF(px, py), 5, 5);
        }
    }

private:
    std::array<Landmarks, 16> landmarks_;
};

class ParticleOverlay : public QWidget
{
public:
    explicit ParticleOverlay(QWidget* parent)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
    }

    void setParticles(const std::vector<Particle>& particles)
    {
        particles_ = particles;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        const double scaleX = width() / (StadiumInfo::A + 2 * StadiumInfo::K);
        const double scaleY = height() / (StadiumInfo::B + 2 * StadiumInfo::K);

        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(255, 0, 0, 150));

        for (const auto& particle : particles_)
        {
            const double px = (particle.x + StadiumInfo::A / 2.0 + StadiumInfo::K) * scaleX;
            const double py = (StadiumInfo::B / 2.0 + StadiumInfo::K - particle.y) * scaleY;
            painter.drawEllipse(QPointF(px, py), 2.0, 2.0);
        }
    }

private:
    std::vector<Particle> particles_;
};

class RobotOverlay : public QWidget
{
public:
    explicit RobotOverlay(QWidget* parent)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
    }

    void setPose(const RobotPose& pose)
    {
        pose_ = pose;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        const double scaleX = width() / (StadiumInfo::A + 2 * StadiumInfo::K);
        const double scaleY = height() / (StadiumInfo::B + 2 * StadiumInfo::K);

        const double px = (pose_.x + StadiumInfo::A / 2.0 + StadiumInfo::K) * scaleX;
        const double py = (StadiumInfo::B / 2.0 + StadiumInfo::K - pose_.y) * scaleY;

        constexpr double ROBOT_WIDTH_PX = 17.0;
        constexpr double ROBOT_HEIGHT_PX = 25.0;
        constexpr double HEADING_LENGTH = 28.0;

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

        const double headingX = px + HEADING_LENGTH * std::cos(pose_.yaw);
        const double headingY = py - HEADING_LENGTH * std::sin(pose_.yaw);

        painter.setPen(QPen(Qt::black, 3));
        painter.drawLine(QPointF(px, py), QPointF(headingX, headingY));

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
    explicit VisionObjectOverlay(QWidget* parent) : QWidget(parent)
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
    void paintEvent(QPaintEvent*) override
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
            if (landmark.type != 0)
                continue;

            const QPointF point = robotToPixel(landmark.x, landmark.y);

            painter.setBrush(QColor(255, 105, 180));
            painter.setPen(QPen(Qt::black, 1));
            painter.drawEllipse(point, 8.0, 8.0);
            painter.drawText(
                point + QPointF(10.0, -10.0),
                QStringLiteral("L%1").arg(landmark.detection_id));
        }

        for (const auto& ball : observation_.ball)
        {
            const QPointF point = robotToPixel(ball.x, ball.y);

            painter.setBrush(QColor(255, 140, 0));
            painter.setPen(QPen(Qt::black, 1));
            painter.drawEllipse(point, 14.0, 14.0);
            painter.drawText(point + QPointF(11.0, -8.0), QStringLiteral("Ball"));
        }

        painter.setBrush(QColor(125, 70, 20));
        painter.setPen(QPen(Qt::black, 1));

        for (const auto& obstacle : observation_.obstacles)
        {
            const QPointF point = robotToPixel(obstacle.x, obstacle.y);
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

        return QPointF(
            (world_x + StadiumInfo::A / 2.0 + StadiumInfo::K) * scale_x,
            (StadiumInfo::B / 2.0 + StadiumInfo::K - world_y) * scale_y);
    }

    VisionObservation observation_;
    RobotPose robot_pose_;
};

// MainWindow 구현
// ============================================================

MainWindow::MainWindow() = default;

MainWindow::~MainWindow()
{
    // Overlay들은 stadiumLabel의 자식이므로 window_ 삭제 시 함께 해제됨
    delete window_;
}

bool MainWindow::load()
{
    QUiLoader loader;

    const auto shareDirectory = ament_index_cpp::get_package_share_directory("localization");
    const QString uiPath = QString::fromStdString(shareDirectory + "/Localization.ui");

    QFile file(uiPath);

    if (!file.open(QFile::ReadOnly))
    {
        loadError_ = QStringLiteral("UI 파일 열기 실패: %1").arg(file.errorString());
        qWarning() << "ERROR:" << loadError_;
        return false;
    }

    QDir::setCurrent(QFileInfo(uiPath).absolutePath());

    window_ = loader.load(&file);
    file.close();

    if (!window_)
    {
        loadError_ = QStringLiteral("UI 로드 실패: %1").arg(loader.errorString());
        qWarning() << "ERROR:" << loadError_;
        return false;
    }

    // QLabel 검색
    stadiumLabel_ = window_->findChild<QLabel*>("label");

    if (!stadiumLabel_)
    {
        loadError_ = QStringLiteral("QLabel 'label'을 찾을 수 없습니다.");
        qWarning() << "ERROR:" << loadError_;
        delete window_;
        window_ = nullptr;
        return false;
    }

    if (auto* visionLogLabel = window_->findChild<QLabel*>("visionlog"))
    {
        visionLogLabel->setText(QStringLiteral("Waiting for /vision_packages..."));
    }

    return true;
}

void MainWindow::show()
{
    if (window_)
    {
        window_->show();
    }
}

void MainWindow::initOverlays(const std::array<Landmarks, 16>& landmarks, const std::vector<Particle>& particles, const RobotPose& initial_pose)
{
    if (!window_ || !stadiumLabel_)
    {
        return;
    }

    landmarkOverlay_ = new LandmarkOverlay(stadiumLabel_, landmarks);
    landmarkOverlay_->setGeometry(stadiumLabel_->rect());
    landmarkOverlay_->show();

    particleOverlay_ = new ParticleOverlay(stadiumLabel_);
    particleOverlay_->setGeometry(stadiumLabel_->rect());
    particleOverlay_->setParticles(particles);
    particleOverlay_->show();

    robotOverlay_ = new RobotOverlay(stadiumLabel_);
    robotOverlay_->setGeometry(stadiumLabel_->rect());
    robotOverlay_->setPose(initial_pose);
    robotOverlay_->show();

    visionObjectOverlay_ = new VisionObjectOverlay(stadiumLabel_);
    visionObjectOverlay_->setGeometry(stadiumLabel_->rect());
    VisionObservation waitingObservation;
    visionObjectOverlay_->setObservation(waitingObservation, initial_pose);
    visionObjectOverlay_->show();
}

void MainWindow::updateRobotPose(const RobotPose& pose)
{
    if (robotOverlay_)
    {
        robotOverlay_->setPose(pose);
    }
}

void MainWindow::updateParticles(const std::vector<Particle>& particles)
{
    if (particleOverlay_)
    {
        particleOverlay_->setParticles(particles);
    }
}

void MainWindow::updateVisionObject(const VisionObservation& obs, const RobotPose& pose)
{
    if (visionObjectOverlay_)
    {
        visionObjectOverlay_->setObservation(obs, pose);
    }
}

namespace
{
void setLabelText(QWidget* root, const char* name, const QString& text)
{
    if (!root)
    {
        return;
    }

    auto* label = root->findChild<QLabel*>(name);
    if (!label)
    {
        return;
    }

    label->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    label->setMargin(12);
    label->setWordWrap(true);
    label->setText(text);
}
}

void MainWindow::setOdometryLog(const QStringList& lines)
{
    setLabelText(window_, "odometrylog", lines.join('\n'));
}

void MainWindow::setImuLog(const QStringList& lines)
{
    setLabelText(window_, "imulog", lines.join('\n'));
}

void MainWindow::setVisionLog(const QString& text)
{
    setLabelText(window_, "visionlog", text);
}