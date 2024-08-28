#pragma once

#include <frc/geometry/Translation3d.h>

namespace yagsl
{
    /** Object with significant mass that needs to be taken into account. */
    class Matter
    {
    public:
        /** Position in meters from robot center in 3d space. */
        frc::Translation3d position;

        /** Mass in kg of object. */
        double mass;

        /**
         * Construct an object representing some significant matter on the robot.
         *
         * @param position Position of the matter in meters.
         * @param mass Mass in kg.
         */
        Matter(frc::Translation3d position, double mass);

        /**
         * Get the center mass of the object.
         *
         * @return center mass = position * mass
         */
        frc::Translation3d MassMoment() const;
    };
} // namespace yagsl