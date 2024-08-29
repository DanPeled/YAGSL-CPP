#pragma once

#include <frc/Timer.h>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <frc/geometry/Rotation2d.h>

namespace yagsl
{
    /** Class to hold simulation data for SwerveModule */
    class SwerveModuleSimulation
    {

    public:
        /** Create simulation class and initialize module at 0. */
        SwerveModuleSimulation();

        /**
         * Update the position and state of the module. Called from
         * SwerveModule::setDesiredState function when simulated.
         *
         * @param desiredState State the swerve module is set to.
         */
        void UpdateStateAndPosition(frc::SwerveModuleState desiredState);

        /**
         * Get the simulated swerve module position.
         *
         * @return SwerveModulePosition of the simulated module.
         */
        frc::SwerveModulePosition GetPosition() const;

        /**
         * Get the SwerveModuleState of the simulated module.
         *
         * @return SwerveModuleState of the simulated module.
         */
        frc::SwerveModuleState GetState() const;

    private:
        /** Main timer to simulate the passage of time. */
        frc::Timer m_timer;
        /** Time delta since last update */
        units::time::second_t m_dt;
        /** Fake motor position. */
        units::meter_t m_fakePos;
        /**
         * The fake speed of the previous state, used to calculate the fake position of the module.
         */
        units::meters_per_second_t m_fakeSpeed;
        /** Last time queried. */
        units::time::second_t m_lastTime;
        /** Current simulated swerve module state. */
        frc::SwerveModuleState m_state;
    };
} // namespace yagsl
