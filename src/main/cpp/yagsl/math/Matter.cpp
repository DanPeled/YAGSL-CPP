#include "yagsl/math/Matter.h"

yagsl::Matter::Matter(frc::Translation3d position, double mass) : position(position), mass(mass) {}

frc::Translation3d yagsl::Matter::MassMoment() const
{
    return position * mass;
}