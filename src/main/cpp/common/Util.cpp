#include "common/Util.h"
#include "Constants.h"

double Util::DegreesToRadians(double theta)
{
    return theta * (2 * std::numbers::pi) / 360.0;
}

double Util::RadiansToDegrees(double theta)
{
    return theta * 360.0 / (2 * std::numbers::pi);
}

// Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
double Util::ZeroTo2PiRads(double theta)
{
    theta = fmod(theta, 2 * std::numbers::pi);
    if (theta < 0)
        theta += 2 * std::numbers::pi;
        
    return theta;
}

// Convert any angle theta in degrees to its equivalent on the interval [0, 360]
double Util::ZeroTo360Degs(double theta)
{
    theta = fmod(theta, 360.0);
    if (theta < 0)
        theta += 360.0;
        
    return theta;
}

// Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
double Util::NegPiToPiRads(double theta)
{
    theta = ZeroTo2PiRads(theta);
    if (theta > std::numbers::pi)
        theta -= 2 * std::numbers::pi;
    else if (theta < -1.0 * std::numbers::pi)
        theta += 2 * std::numbers::pi;
    
    return theta;
}

double Util::GetAverage(std::vector<double> numbers)
{
    double sum = 0.0;

    for (auto n : numbers)
        sum += n;
    
    return sum / numbers.size();
}