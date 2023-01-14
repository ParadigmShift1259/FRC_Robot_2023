#pragma once

#include <string>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>

using namespace std;
using namespace rev;

class PIDLoaderNEO
{
public:
    PIDLoaderNEO(string name, bool adjustable, double p, double i, double d, double iz, double ia);
    PIDLoaderNEO(string name, bool adjustable, double p, double i, double d, double iz, double ia, double ff, double max, double min);

    /// Loads turn PID controller with values, also sends default PID values to SmartDashboard
    /// \param turnPIDController        The SparkMaxPIDController of the CANSparkMax responsible for turning
    void Load(SparkMaxPIDController& turnPIDController);
    
    /// Loads turn PID controller with on the fly values from SmartDashboard
    /// \param turnPIDController        The SparkMaxPIDController of the CANSparkMax responsible for turning
    void LoadFromNetworkTable(SparkMaxPIDController& turnPIDController);

private:
    string m_name;
    bool m_adjustable;

    double m_p;
    double m_i;
    double m_d;
    double m_iz;
    double m_ia;
    double m_ff;
    double m_max = 1.0;
    double m_min = -1.0;
};