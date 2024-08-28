#pragma once
#include "frc/controller/PIDController.h"
#include "PIDFRange.h"
namespace yagsl
{
    class PIDFConfig
    {
    public:
        /** Proportional Gain for PID. */
        double p;
        /** Integral Gain for PID. */
        double i;
        /** Derivative Gain for PID. */
        double d;
        /** feedforward value for pid. */
        double f;
        /** Integral zone of the PID. */
        double iz;

        PIDFRange output;

        /**
         * PIDF Config constructor to contain the values.
         *
         * @param p P gain.
         * @param i I gain.
         * @param d D gain.
         * @param f F gain.
         * @param iz Intergral zone.
         */
        PIDFConfig(double p, double i, double d, double f, double iz);

        /**
         * PIDF Config constructor to contain the values.
         *
         * @param p P gain.
         * @param i I gain.
         * @param d D gain.
         * @param f F gain.
         */
        PIDFConfig(double p, double i, double d, double f);

        /**
         * PIDF Config constructor to contain the values.
         *
         * @param p P gain.
         * @param i I gain.
         * @param d D gain.
         */
        PIDFConfig(double p, double i, double d);

        /**
         * PIDF Config constructor to contain the values.
         *
         * @param p P gain.
         * @param d D gain.
         */
        PIDFConfig(double p, double d);

        frc::PIDController createPIDController()
        {
            return frc::PIDController(p, i, d);
        }
    };
}