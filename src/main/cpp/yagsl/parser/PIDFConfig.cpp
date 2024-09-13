#include "yagsl/parser/PIDFConfig.h"
using namespace yagsl::parser;

PIDFConfig::PIDFConfig(double p, double i, double d, double f, double iz)
    : p(p), i(i), d(d), f(f), iz(iz)
{
}

PIDFConfig::PIDFConfig(double p, double i, double d, double f) : PIDFConfig(p, i, d, f, 0.0)
{
}

PIDFConfig::PIDFConfig(double p, double i, double d) : PIDFConfig(p, i, d, 0.0, 0.0)
{
}

PIDFConfig::PIDFConfig(double p, double d) : PIDFConfig(p, 0.0, d, 0.0, 0.0)
{
}