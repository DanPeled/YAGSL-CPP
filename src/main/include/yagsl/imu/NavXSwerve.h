#pragma once

#include "SwerveIMU.h"
#include "AHRS.h"
#include "yagsl/telemetry/Alert.h"
#include <frc/SerialPort.h>

namespace yagsl
{
    class NavXSwerve : public SwerveIMU
    {
    public:
        /**
         * Constructor for the NavX swerve.
         *
         * @param port Serial Port to connect to.
         */
        NavXSwerve(frc::SerialPort::Port port);

        /**
         * Constructor for the NavX swerve.
         *
         * @param port SPI Port to connect to.
         */
        NavXSwerve(frc::SPI::Port port);

        /**
         * Constructor for the NavX swerve.
         *
         * @param port I2C Port to connect to.
         */
        NavXSwerve(frc::I2C::Port port);

        /** Reset IMU to factory default. */
        void FactoryDefault() override;

        /** Clear sticky faults on IMU. */
        void ClearStickyFaults() override;

        /**
         * Set the gyro offset.
         *
         * @param offset gyro offset as a Rotation3d.
         */
        void SetOffset(frc::Rotation3d offset);

        void SetInverted(bool invertIMU) override;

        frc::Rotation3d GetRawRotation3d() override;

        frc::Rotation3d GetRotation3d() override;

        std::optional<frc::Translation3d> GetAccel() override;

        void *GetIMU() override;

    private:
        /** NavX IMU. */
        AHRS m_gyro;
        /** Offset for the NavX. */
        frc::Rotation3d m_offset;
        /** Inversion for the gyro */
        bool m_invertedIMU = false;
        /** An Alert for if there is an error instantiating the NavX. */
        Alert m_navXError;
    };
} // namespace yagsl
