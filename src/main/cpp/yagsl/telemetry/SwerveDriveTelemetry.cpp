#include "yagsl/telemetry/SwerveDriveTelemetry.h"

namespace yagsl
{
    const Alert SwerveDriveTelemetry::canIdWarning{
        "JSON", "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40",
        Alert::AlertType::WARNING};

    const Alert SwerveDriveTelemetry::i2cLockupWarning{
        "IMU", "I2C lockup issue detected on roboRIO. Check console for more information.",
        Alert::AlertType::WARNING};

    const Alert SwerveDriveTelemetry::serialCommsIssueWarning{
        "IMU", "Serial comms is interrupted with USB and other serial traffic and causes intermittent connected/disconnection issues. Please consider another protocol or be mindful of this.",
        Alert::AlertType::WARNING};

    SwerveDriveTelemetry::TelemetryVerbosity SwerveDriveTelemetry::verbosity = SwerveDriveTelemetry::TelemetryVerbosity::MACHINE;

    bool SwerveDriveTelemetry::isSimulation = frc::RobotBase::IsSimulation();

    std::string SwerveDriveTelemetry::rotationUnit = "degrees";
    std::string SwerveDriveTelemetry::forwardDirection = "up";

    void SwerveDriveTelemetry::UpdateData()
    {
        frc::SmartDashboard::PutNumber("swerve/moduleCount", moduleCount);
        frc::SmartDashboard::PutNumberArray("swerve/wheelLocations", wheelLocations);
        frc::SmartDashboard::PutNumberArray("swerve/measuredStates", measuredStates);
        frc::SmartDashboard::PutNumberArray("swerve/desiredStates", desiredStates);
        frc::SmartDashboard::PutNumber("swerve/robotRotation", robotRotation);
        frc::SmartDashboard::PutNumber("swerve/maxSpeed", maxSpeed);
        frc::SmartDashboard::PutString("swerve/rotationUnit", rotationUnit);
        frc::SmartDashboard::PutNumber("swerve/sizeLeftRight", sizeLeftRight);
        frc::SmartDashboard::PutNumber("swerve/sizeFrontBack", sizeFrontBack);
        frc::SmartDashboard::PutString("swerve/forwardDirection", forwardDirection);
        frc::SmartDashboard::PutNumber("swerve/maxAngularVelocity", maxAngularVelocity);
        frc::SmartDashboard::PutNumberArray("swerve/measuredChassisSpeeds", measuredChassisSpeeds);
        frc::SmartDashboard::PutNumberArray("swerve/desiredChassisSpeeds", desiredChassisSpeeds);
    }
} // namespace yagsl
