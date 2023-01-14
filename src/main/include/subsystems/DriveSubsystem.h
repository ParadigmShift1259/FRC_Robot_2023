/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "SwerveModule.h"

#include <vector>
#include <wpi/array.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#define USE_SWERVE_POSE_ESTIMATOR
#ifdef USE_SWERVE_POSE_ESTIMATOR
#include <frc/estimator/SwerveDrivePoseEstimator.h>
using SwerveOdo = frc::SwerveDrivePoseEstimator<4>;
#else
#include <frc/kinematics/SwerveDriveOdometry.h>
using SwerveOdo = SwerveDriveOdometry<4>;
#endif

#include <frc/trajectory/Trajectory.h>
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/Encoder.h>

#include <units/time.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/CANifier.h>

#include "common/Util.h"
#include "Gyro.h"
#include "IOdometry.h"

#include "Constants.h"


// Uncomment to directly set states to each module
//#define MANUAL_MODULE_STATES
// Uncomment to tune Rotation Drive PIDs
//#define TUNE_ROTATION_DRIVE

using namespace ctre::phoenix;
// using namespace DriveConstants;
// using namespace std;
// using namespace frc;

class DriveSubsystem : public frc2::SubsystemBase
{
public:
    enum ModuleLocation    //!< Order as returned by kDriveKinematics.ToSwerveModuleStates
    {
        kFrontLeft,
        kFrontRight,
        kRearLeft,
        kRearRight
    };

    DriveSubsystem(Team1259::Gyro *gyro, IOdometry& odo);
    ~DriveSubsystem() noexcept {};

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    bool m_enabled = false;

    // Subsystem methods go here.

    /// Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
    /// and the linear speeds have no effect on the angular speed.
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param rot           Angular rate of the robot.
    /// \param fieldRelative Whether the provided x and y speeds are relative to the field.
    /// \param isAuto          Whether the bot is using function for auto or not. False by default. 
    void Drive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rot, bool fieldRelative);

    // Drives the robot with the right stick controlling the position angle of the robot
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param rot           Angle of the robot in radians
    /// \param fieldRelative Whether the provided translational speeds are relative to the field.
    void RotationDrive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radian_t rot, bool fieldRelative);

    // Drives the robot with the right stick controlling the position angle of the robot
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param xRot          Angle of the robot on the x axis
    /// \param yRot          Angle of the robot on the y axis
    /// \param fieldRelative Whether the provided translational speeds are relative to the field.
    void RotationDrive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, double xRot, double yRot, bool fieldRelative);

    /// Drives the robot and maintains robot angle with no rotational input
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param rot           Angular rate of the robot.
    /// \param fieldRelative Whether the provided x and y speeds are relative to the field.
    void HeadingDrive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rot, bool fieldRelative);

    /// Updates the last heading set for Heading Drive. Needs to be called if transitioning from other Drive functions to HeadingDrive
    void UpdateLastHeading();

    /// Resets the drive encoders to currently read a position of 0.
    void ResetEncoders();

    /// Readable alias for array of swerve modules
    // using SwerveModuleStates = wpi::array<frc::SwerveModuleState, DriveConstants::kNumSwerveModules>;
    using SwerveModuleStates = wpi::array<frc::SwerveModuleState, DriveConstants::kNumSwerveModules>;
    /// Sets the drive SpeedControllers to a power from -1 to 1.
    void SetModuleStates(SwerveModuleStates desiredStates);

    /// Returns the currently-estimated pose of the robot.
    /// \return The pose.
    Pose2d GetPose();
    StateHist GetState() const;

    /// Returns the previous estimated pose of the robot at the given timestamp.
    /// \return The pose.
    Pose2d GetPose(units::time::second_t timestamp) const;
    StateHist GetState(units::time::second_t timestamp) const;

    units::meters_per_second_t GetSpeed(void) const;

    /// Converts PWM input on the CANifier to a pulse width
    /// \param pwmChannel The PWM channel to pass in
    /// \return The pulse width of the PWM channel
    double PWMToPulseWidth(CANifier::PWMChannel pwmChannel);

    /// Resets the odometry to the specified pose.
    /// \param pose The pose to which to set the odometry.
    void ResetOdometry(Pose2d pose);

    /// Set all 4 wheels to the zero position
    void WheelsForward();

    /// Resync all relative NEO turn encoders to the absolute encoders
    void ResetRelativeToAbsolute();

    void SetMaxDriveSpeed(meters_per_second_t maxDriveSpeed) { m_maxDriveSpeed = maxDriveSpeed; }
    meters_per_second_t GetYvelocity() const { return m_yVelocity; }

    const StateHistColl& GetStateHist() const { return m_StateHist; }

    void AddVisionMeasurement(const Pose2d& visionRobotPose, units::second_t timestamp) {m_odometry.AddVisionMeasurement(visionRobotPose, timestamp); }
    void SetVisionMeasurementStdDevs( const wpi::array<double, 3>& visionMeasurementStdDevs) {m_odometry.SetVisionMeasurementStdDevs(visionMeasurementStdDevs); } 

    /// The kinematics object converts inputs into 4 individual swerve module turn angle and wheel speeds
    SwerveDriveKinematics<kNumSwerveModules> kDriveKinematics{
        Translation2d( kWheelBase / 2,  kTrackWidth / 2),    // +x, +y FL
        Translation2d( kWheelBase / 2, -kTrackWidth / 2),    // +x, -y FR
        Translation2d(-kWheelBase / 2,  kTrackWidth / 2),    // -x, +y RL
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2)};   // -x, -y RR

    bool OdoValid();

// made public just to allow trial change of current limit from Robot teleopinit
SwerveModule m_frontLeft;
SwerveModule m_frontRight;
SwerveModule m_rearRight;
SwerveModule m_rearLeft;

private:    
    /// Get all 4 swerve module wheel speed to update the odometry with
    SwerveModuleStates getCurrentWheelSpeeds()
    {
        SwerveModuleStates sms{
            m_frontLeft.GetState(),
            m_frontRight.GetState(),
            m_rearLeft.GetState(),
            m_rearRight.GetState()
        };
        return sms;
    }

    /// \name Swerve Modules
    /// The drive subsystem owns all 4 swerve modules
    ///@{
// SwerveModule m_frontLeft;
// SwerveModule m_frontRight;
// SwerveModule m_rearRight;
// SwerveModule m_rearLeft;
    ///@}

    /// Reads the absolute encoder pulse widths
    CANifier m_canifier;
    /// Gyro to determine field relative driving, from @ref RobotContainer
    Team1259::Gyro *m_gyro;
    /// Odometry class for tracking robot pose
    SwerveOdo m_odometry;
    bool m_odoValid;

    /// PID to control overall robot chassis rotation 
    frc2::PIDController m_rotationPIDController{
        DriveConstants::kRotationDriveP,
        DriveConstants::kRotationDriveI,
        DriveConstants::kRotationDriveD
    };
    /// Last maintained heading, used for @ref HeadingDrive
    double m_lastHeading;
    /// Whether or not rotation input was provided, used for @ref HeadingDrive
    bool m_rotationalInput;
    Timer m_timer;
    StateHistColl m_StateHist;
    IOdometry& m_odo;
    double m_velocity;
    double m_acceleration;
    meters_per_second_t m_maxDriveSpeed { kDriveSpeed };
    meters_per_second_t m_yVelocity {0.0};

    // wpi::array<SwerveModulePosition, 4> = // TODO

    wpi::log::DoubleLogEntry m_logRobotPoseX;
    wpi::log::DoubleLogEntry m_logRobotPoseY;
    wpi::log::DoubleLogEntry m_logRobotPoseTheta;
    wpi::log::DoubleLogEntry m_logRobotSpeed;
    wpi::log::DoubleLogEntry m_logRobotAccel;
    wpi::log::DoubleLogEntry m_logTurretAngle;
};
