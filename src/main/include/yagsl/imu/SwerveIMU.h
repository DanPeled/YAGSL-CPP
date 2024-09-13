#pragma once

#include <optional>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation3d.h>

namespace yagsl
{
    class SwerveIMU
    {
    public:
        /** Reset IMU to factory default. */
        virtual void FactoryDefault() = 0;

        /** Clear sticky faults on IMU. */
        virtual void ClearStickyFaults() = 0;

        /**
         * Set the gyro offset.
         *
         * @param offset gyro offset as a Rotation3d.
         */
        virtual void SetOffset(frc::Rotation2d offset) = 0;

        /**
         * Set the gyro to invert its default direction.
         *
         * @param invertIMU gyro direction
         */
        virtual void SetInverted(bool invertIMU) = 0;

        /**
         * Fetch the Rotation3d from the IMU without any zeroing. Robot relative.
         *
         * @return Rotation3d from the IMU.
         */
        virtual frc::Rotation3d GetRawRotation3d() = 0;

        /**
         * Fetch the Rotation3d from the IMU. Robot relative.
         *
         * @return Rotation3d from the IMU.
         */
        virtual frc::Rotation3d GetRotation3d() = 0;

        /**
         * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration
         * isn't supported returns empty.
         *
         * @return Translation3d of the acceleration as an Optional.
         */
        virtual std::optional<frc::Translation3d> GetAccel() = 0;

        /**
         * Get the instantiated IMU object.
         *
         * @return IMU object.
         */
        virtual void *GetIMU() = 0;
    };
} // namespace yagsl
