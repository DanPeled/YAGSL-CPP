
#include "yagsl/simulation/SwerveModuleSimulation.h"

namespace yagsl
{
    SwerveModuleSimulation::SwerveModuleSimulation() : m_timer{}
    {
        m_timer.Start();
        m_lastTime = m_timer.Get();
        m_state = frc::SwerveModuleState(0_mps, frc::Rotation2d(0_deg));
        m_fakeSpeed = 0_mps;
        m_fakePos = 0_m;
        m_dt = 0_s;
    }

    void SwerveModuleSimulation::UpdateStateAndPosition(frc::SwerveModuleState desiredState)
    {
        m_dt = m_timer.Get() - m_lastTime;
        m_lastTime = m_timer.Get();

        m_state = desiredState;
        m_fakeSpeed = desiredState.speed;
        m_fakePos += m_fakeSpeed * m_dt;
    }

    frc::SwerveModulePosition SwerveModuleSimulation::GetPosition() const
    {
        return frc::SwerveModulePosition(m_fakePos, m_state.angle);
    }

    frc::SwerveModuleState SwerveModuleSimulation::GetState() const
    {
        return m_state;
    }
} // namespace yagsl
