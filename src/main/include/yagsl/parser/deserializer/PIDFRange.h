#pragma once

namespace yagsl::parser
{
    /** Integral zone of the PID. */
    struct PIDFRange
    {
    public:
        /** Integral zone of the PID. */
        double min = -1;
        /** Integral zone of the PID. */
        double max = 1;
    };
} // namespace yagsl
