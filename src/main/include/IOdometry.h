#pragma once

#include <vector>

#include <wpi/array.h>

#include <frc/geometry/Pose2d.h>
// #include <frc/trajectory/Trajectory.h>

#include <units/time.h>

// #include "frc/StateSpaceUtil.h"
#include "StateWithTurretAngle.h"

//using StateHistColl = std::vector<frc::Trajectory::State>;
//using StateHist = frc::Trajectory::State;
using StateHist = StateWithTurretAngle;
using StateHistColl = std::vector<StateHist>;

using namespace units;

class IOdometry
{
public:

    virtual frc::Pose2d GetPose() = 0;
    virtual frc::Pose2d GetPose(second_t timestamp) const = 0;
    virtual StateHist GetState() const = 0;
    virtual StateHist GetState(second_t timestamp) const = 0;
    virtual const StateHistColl& GetStateHist() const = 0;
    //virtual degree_t GetTurretAngle() = 0;
    //virtual bool HasAutoRun() = 0;
    virtual bool OdoValid() = 0;
    virtual void ResetOdometry(frc::Pose2d) = 0;
    //virtual void AddVisionMeasurement(const frc::Pose2d& visionRobotPose, second_t timestamp) = 0;
    //virtual void SetVisionMeasurementStdDevs( const wpi::array<double, 3>& visionMeasurementStdDevs) = 0; 
};
