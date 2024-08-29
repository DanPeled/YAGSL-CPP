#pragma once

#include <frc/Timer.h>

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose2d.h>

#include <frc/smartdashboard/Field2d.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "yagsl/telemetry/SwerveDriveTelemetry.h"

namespace yagsl
{
    /** Simulation for SwerveDrive IMU. */
    class SwerveIMUSimulation
    {
    public:
        SwerveIMUSimulation();

        /**
         * Get the estimated angle of the robot.
         *
         * @return Rotation2d estimation of the robot.
         */
        frc::Rotation2d GetYaw();

        /**
         * Pitch is not simulated currently, always returns 0.
         *
         * @return Pitch of the robot as Rotation2d.
         */
        frc::Rotation2d GetPitch();

        /**
         * Roll is not simulated currently, always returns 0.
         *
         * @return Roll of the robot as Rotation2d.
         */
        frc::Rotation2d GetRoll();

        /**
         * Gets the estimated gyro Rotation3d of the robot.
         *
         * @return The heading as a Rotation3d angle
         */
        frc::Rotation3d GetGyroRotation3d();

        /**
         * Fetch the acceleration [x, y, z] from the IMU in m/s/s. If acceleration isn't supported
         * returns empty.
         *
         * @return Translation3d of the acceleration as an Optional}.
         */
        std::optional<frc::Translation3d> GetAccel() const;

        /**
         * Set the heading of the robot.
         *
         * @param angle Angle of the robot in radians.
         */
        void SetAngle(units::angle::degree_t angle);

        /**
         * Update the odometry of the simulated {@link swervelib.SwerveDrive} and post the {@link
         * swervelib.SwerveModule} states to the {@link Field2d}.
         *
         * @param kinematics {@link SwerveDriveKinematics} of the swerve drive.
         * @param states {@link SwerveModuleState} array of the module states.
         * @param modulePoses {@link Pose2d} representing the swerve modules.
         * @param field {@link Field2d} to update.
         */
        void UpdateOdometry(frc::SwerveDriveKinematics<4> kinematics, wpi::array<frc::SwerveModuleState, 4> states, std::span<frc::Pose2d> modulePoses, frc::Field2d &field);

    private:
        /** Main timer to control movement estimations. */
        frc::Timer m_timer;
        /** The last time the timer was read, used to determine position changes. */
        units::time::second_t m_lastTime;
        /** Main timer to control movement estimations. */
        units::angle::degree_t m_angle;
    };
} // namespace yagsl