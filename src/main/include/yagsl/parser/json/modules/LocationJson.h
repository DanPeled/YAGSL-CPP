#pragma once

namespace yagsl::parser::json::modules
{
    /**
     * Location JSON parsed class. Used to access the JSON data. Module locations, in inches, as
     * distances to the center of the robot. +x is towards the robot front, and +y is towards robot
     * left.
     */
    class LocationJson
    {
    public:
        double front = 0, x = 0;
        double left = 0, y = 0;
    };
} // namespace yagsl::parser::json::modules