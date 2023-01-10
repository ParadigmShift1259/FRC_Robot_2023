#include "commands/HomeTarget.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

HomeTarget::HomeTarget(   FlywheelSubsystem *flywheel
                        , TurretSubsystem *turret
                        , HoodSubsystem *hood
                        , VisionSubsystem &vision
                        , bool *turretready
                        , bool *firing
                        , bool *finished
                        , GetYvelocityCallback yVelocityCb)
    : m_flywheel(flywheel)
    , m_turret(turret)
    , m_hood(hood)
    , m_vision(vision)
    , m_turretready(turretready)
    , m_firing(firing)
    , m_finished(finished)
    , m_yVelocityCb(yVelocityCb)
{
    AddRequirements({flywheel, turret, hood});
    *m_turretready = false;
    *m_firing = false;
    *m_finished = false;
}

void HomeTarget::Initialize()
{
#define COMMAND_TIMING
#ifdef COMMAND_TIMING
    m_startTime = m_timer.GetFPGATimestamp().to<double>();
    printf("timestamp start home target %.3f\n", m_startTime);
#endif
    *m_turretready = false;
    *m_firing = false;
    *m_finished = false;
}

void HomeTarget::Execute()
{
    // Homes flywheel, turret, and hood to the right angles through a formula
    SmartDashboard::PutBoolean("TEST_VIS_ACTIVE", m_vision.GetValidTarget());

    if (!m_vision.GetValidTarget())
        return;

    double distToHubCenter = m_vision.GetHubDistance(false);
    double distance = distToHubCenter - kHubOffsetRimToCenter.to<double>();

    // if (std::isnan(distance))
    if (distance != distance)
    {
        printf("Not a Valid Distane\n");
        frc::DataLogManager::Log("Not a Valid Distane");
        return;
    }

    m_hood->SetByDistance(distToHubCenter);
    double flywheelspeed = m_hood->GetFlywheelSpeed();
//#define USE_BLORP_SHOT // For auto practice with no shots
#ifdef USE_BLORP_SHOT
    flywheelspeed = 1000;
#endif
    m_flywheel->SetRPM(flywheelspeed);

    SmartDashboard::PutBoolean("D_FIRE_AT_RPM", m_flywheel->IsAtRPM());
    SmartDashboard::PutBoolean("D_FIRE_AT_SET", m_turret->isAtSetpoint());

    if (m_flywheel->IsAtRPM())
    //if (m_flywheel->IsAtRPM() && m_yVelocityCb() <= 1.1 * kSlowDriveSpeed.to<double>())
    //if (m_flywheel->IsAtRPMPositive() && m_yVelocityCb() <= 1.1 * kSlowDriveSpeed.to<double>())
    //if (m_flywheel->IsAtRPM() && m_turret->isAtSetpoint() && m_yVelocityCb() <= 1.1 * kSlowDriveSpeed.to<double>())
    {
        *m_turretready = true;
        m_vision.CamCapture();
        frc::DataLogManager::Log(fmt::format("Fireing: dist={}, hub angle={}, turret angle={}, RPM={}, hood pos.={}",
                                m_vision.GetHubDistance(false) * 39.37, 
                                m_vision.GetHubAngle() * 180/3.14, 
                                m_turret->GetCurrentAngle(),
                                m_flywheel->GetRPM(),
                                m_hood->GetServoPosition()
                                ));
    }

    frc::SmartDashboard::PutBoolean("TEST_READY_TO_FIRE", *m_turretready);
}

bool HomeTarget::IsFinished()
{
    return *m_turretready;
}

void HomeTarget::End(bool interrupted)
{
    *m_turretready = false;
#ifdef COMMAND_TIMING
    auto endTime = m_timer.GetFPGATimestamp().to<double>();
    auto elapsed = endTime - m_startTime;
    printf("timestamp end home target %.3f elapsed %.3f\n", endTime, elapsed);
#endif
}  
