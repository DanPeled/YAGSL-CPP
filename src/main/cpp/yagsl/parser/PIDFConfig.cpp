#include "yagsl/parser/PIDFConfig.h"

yagsl::PIDFConfig::PIDFConfig(double p, double i, double d, double f, double iz)
    : p(p), i(i), d(d), f(f), iz(iz)
{
}

yagsl::PIDFConfig::PIDFConfig(double p, double i, double d, double f) : PIDFConfig(p, i, d, f, 0.0)
{
}

yagsl::PIDFConfig::PIDFConfig(double p, double i, double d) : PIDFConfig(p, i, d, 0.0, 0.0)
{
}

yagsl::PIDFConfig::PIDFConfig(double p, double d) : PIDFConfig(p, 0.0, d, 0.0, 0.0)
{
}