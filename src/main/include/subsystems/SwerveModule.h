/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>

#include <networktables/NetworkTableEntry.h>

#include <rev/CANSparkMax.h>

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h>

#include "Constants.h"
#include "common/Util.h"
#include "common/PIDLoaderFalcon.h"
#include "common/PIDLoaderNEO.h"
 
// Uncomment this to prevent swerve modules from driving
//#define DISABLE_DRIVE

using namespace rev;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ModuleConstants;
using namespace units;
using namespace std;
using namespace frc;

using GetPulseWidthCallback = function<double (CANifier::PWMChannel)>;


class SwerveModule
{
    using radians_per_second_squared_t = compound_unit<radians, units::inverse<squared<second>>>;

public:
    SwerveModule( int driveMotorChannel
                , int turningMotorChannel
                , GetPulseWidthCallback pulseWidthCallback
                , CANifier::PWMChannel pwmChannel
                , bool driveEncoderReversed
                , double offSet
                , const string& name);

    void Periodic();

    /// Get the state for the swerve module pod
    /// \return             The state (vector with speed and angle) representig the current module state
    /// @todo Currently GetState uses the absolute angle instead of the relative angle that we should be using
    frc::SwerveModuleState GetState();
    
    /// Get the position for the swerve module pod
    /// \return             The posistion (distance and angle) representing the current module position
    /// @todo Currently is a dummy function that returns 0.0 meters as the distance
    frc::SwerveModulePosition GetPosition();

    /// Set the desired state for the swerve module pod
    /// \param state        The state (vector with speed and angle) representing the desired module state
    void SetDesiredState(frc::SwerveModuleState &state);
    /// Resets the drive motor encoders to 0
    void ResetEncoders();
    /// Resync the relative NEO turn encoder to the absolute encoder
    void ResetRelativeToAbsolute();

    TalonFX m_driveMotor; // made public just to allow trial change of current limit from Robot teleopinit
 
private:
    /// Calculate the absolute angle, in radians, based on the pulse widths from @ref m_pulseWidthCallback
    double CalcAbsoluteAngle();
    /// Calculate the MPS of the drive motor based on its current encoder tick velocity
    /// \return             Meters per second
    meters_per_second_t CalcMetersPerSec();
    /// Calculate drive motor encoder ticks based on the MPS
    /// \param speed        Meters per second
    /// \return             Encoder ticks
    double CalcTicksPer100Ms(meters_per_second_t speed);

    /// The offset, in pulse widths, for syncing the relative encoder to the absolute encoder
    double m_offset;
    /// String used to identify each pod, used for SmartDashboard prints
    string m_name;

    /// Falon 500 that drives the pod
//    TalonFX m_driveMotor;
    /// \name NEO that turns the pod, controls angle with relative encoder and PID
    ///@{
    CANSparkMax m_turningMotor;
    SparkMaxAlternateEncoder m_turnRelativeEncoder = m_turningMotor.GetAlternateEncoder(SparkMaxAlternateEncoder::Type::kQuadrature, 
                                                                          ModuleConstants::kTurnEncoderCPR);
    SparkMaxPIDController m_turnPIDController = m_turningMotor.GetPIDController();
    ///@}

    /// PID param loader for the TalonFX
    PIDLoaderFalcon m_drivePIDLoader;
    /// PID param loader for the CANSparkMax
    PIDLoaderNEO m_turnPIDLoader;

    /// Callback used to determine the pulse width
    /// \param pwmChannel       Channel to decide which absolute encoder's pulse width values are retrieved
    GetPulseWidthCallback m_pulseWidthCallback;
    CANifier::PWMChannel m_pwmChannel;

    /// Timer used to sync absolute and relative encoders on robot turn on
    Timer m_timer;
};