#include "yagsl/imu/NavXSwerve.h"

namespace yagsl
{
    NavXSwerve::NavXSwerve(frc::SerialPort::Port port) : m_navXError("IMU", "Error instantiating NavX.", Alert::AlertType::ERROR_TRACE),
                                                         m_gyro{port}
    {
        try
        {
            FactoryDefault();
            frc::SmartDashboard::PutData(&m_gyro);
        }
        catch (const std::exception &ex)
        {
            m_navXError.SetText(std::string("Error instatiating NavX: ") + ex.what());
            m_navXError.Set(true);
        }
    }

    NavXSwerve::NavXSwerve(frc::SPI::Port port) : m_gyro{port}
    {
        try
        {
            /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
            FactoryDefault();
            frc::SmartDashboard::PutData(&m_gyro);
        }
        catch (const std::exception &ex)
        {
            m_navXError.SetText(std::string("Error instatiating NavX: ") + ex.what());
            m_navXError.Set(true);
        }
    }

    NavXSwerve::NavXSwerve(frc::I2C::Port port) : m_gyro{port}
    {
        try
        {
            /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
            FactoryDefault();
            frc::SmartDashboard::PutData(&m_gyro);
        }
        catch (const std::exception &ex)
        {
            m_navXError.SetText(std::string("Error instantiating NavX: ") + ex.what());
            m_navXError.Set(true);
        }
    }

    void NavXSwerve::FactoryDefault()
    {
        // gyro.reset(); // Reported to be slow
        m_offset = m_gyro.GetRotation3d();
    }

    void NavXSwerve::ClearStickyFaults() {}

    void NavXSwerve::SetOffset(frc::Rotation3d offset)
    {
        m_offset = offset;
    }

    void NavXSwerve::SetInverted(bool invertIMU)
    {
        m_invertedIMU = invertIMU;
    }

    frc::Rotation3d NavXSwerve::GetRawRotation3d()
    {
        return m_invertedIMU ? -m_gyro.GetRotation3d() : m_gyro.GetRotation3d();
    }

    frc::Rotation3d NavXSwerve::GetRotation3d()
    {
        return GetRawRotation3d() - m_offset;
    }

    std::optional<frc::Translation3d> NavXSwerve::GetAccel()
    {
        return std::optional<frc::Translation3d>{
            frc::Translation3d(
                units::length::meter_t{m_gyro.GetWorldLinearAccelX()},
                units::length::meter_t{m_gyro.GetWorldLinearAccelY()},
                units::length::meter_t{m_gyro.GetWorldLinearAccelZ()}) *
            9.81};
    }

    void *NavXSwerve::GetIMU()
    {
        return &m_gyro;
    }
} // namespace yagsl
