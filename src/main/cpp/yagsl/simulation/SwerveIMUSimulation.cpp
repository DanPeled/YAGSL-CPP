#include "yagsl/simulation/SwerveIMUSimulation.h"

namespace yagsl
{
    SwerveIMUSimulation::SwerveIMUSimulation() : m_timer{}
    {
        m_timer.Start();
        m_lastTime = m_timer.Get();
    }

    frc::Rotation2d SwerveIMUSimulation::GetYaw()
    {
        return frc::Rotation2d(m_angle);
    }

    frc::Rotation2d SwerveIMUSimulation::GetPitch()
    {
        return frc::Rotation2d();
    }

    frc::Rotation2d SwerveIMUSimulation::GetRoll()
    {
        return frc::Rotation2d();
    }

    frc::Rotation3d SwerveIMUSimulation::GetGyroRotation3d()
    {
        return frc::Rotation3d(0.0_rad, 0.0_rad, units::angle::radian_t{m_angle});
    }

    std::optional<frc::Translation3d> SwerveIMUSimulation::GetAccel() const
    {
        return std::optional<frc::Translation3d>{};
    }

    void SwerveIMUSimulation::SetAngle(units::angle::degree_t angle)
    {
        m_angle = angle;
    }

    void SwerveIMUSimulation::UpdateOdometry(frc::SwerveDriveKinematics<4> kinematics, wpi::array<frc::SwerveModuleState, 4> states, std::span<frc::Pose2d> modulePoses, frc::Field2d &field)
    {
        m_angle += kinematics.ToChassisSpeeds(states).omega * (m_timer.Get() - m_lastTime);
        m_lastTime = m_timer.Get();
        field.GetObject("XModules")->SetPoses(modulePoses);
    }
} // namespace yagsl
