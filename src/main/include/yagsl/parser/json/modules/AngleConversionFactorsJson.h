#pragma once

namespace yagsl::parser::json::modules
{
    /** Angle motor conversion factors composite JSON parse class. */
    class AngleConversionFactorsJson
    {
    public:
        /**
         * Gear ratio for the angle/steering/azimuth motor on the Swerve Module. Motor rotations to 1
         * wheel rotation.
         */
        double gearRatio = 0;
        /** Calculated or given conversion factor. */
        double factor = 0;

        /**
         * Calculate the drive conversion factor.
         *
         * @return Drive conversion factor, if factor isn't set.
         */
        double calculate();
    };
} // namespace yagsl
