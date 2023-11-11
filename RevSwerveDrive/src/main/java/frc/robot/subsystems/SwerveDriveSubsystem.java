// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlSytemCanIdConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MotorCanIdConstants;
import frc.robot.Constants.OperatorConstants;
import frc.utils.AutoUtils;
import frc.utils.OperatorUtils;

public class SwerveDriveSubsystem extends SubsystemBase {
  double maxDrivetrainMPS = DriveTrainConstants.kMaxSpeedMetersPerSecond;
  double driveTrainOrientationP = 0.05, driveTrainOrientationI = 0, driveTrainOrientationD = 0;
  double ballanceP = -0.0075, ballanceI = 0, ballanceD = 0;
  double driveTrainPositionP = 1.0, driveTrainPositionI = 0, driveTrainPositionD = 0;
  boolean angleTargetingMode = true;
  double yawOffset = 0;
  double rollOffset = 0;
  double pitchOffset = 0;
  boolean fieldRelativeControlState = true;
  double angleTargetDegrees = 0;
  XboxController m_driverController;
  PIDController orientationPID = new PIDController(driveTrainOrientationP, driveTrainOrientationI,
      driveTrainOrientationD);
  PIDController ballancePID = new PIDController(ballanceP, ballanceI, ballanceD);
  PIDController positionPID = new PIDController(driveTrainPositionP, driveTrainPositionI, driveTrainPositionD);
  boolean onPlatform = false;
  double wheelEncoderAtRampStart = 0;
  double timeLastDisabled = Timer.getFPGATimestamp();

  public SwerveDriveSubsystem(XboxController controller) {
    m_driverController = controller;
    orientationPID.enableContinuousInput(-180, 180);
    SmartDashboard.putNumber("maxDrivetrainMPS",  maxDrivetrainMPS);
    SmartDashboard.putNumber("driveTrainOrientationP", driveTrainOrientationP);
    SmartDashboard.putNumber("driveTrainOrientationI", driveTrainOrientationI);
    SmartDashboard.putNumber("driveTrainOrientationD", driveTrainOrientationD);
    SmartDashboard.putNumber("ballanceP", ballanceP);
    SmartDashboard.putNumber("ballanceI", ballanceI);
    SmartDashboard.putNumber("ballanceD", ballanceD);
    SmartDashboard.putNumber("driveTrainPositionP", driveTrainPositionP);
    SmartDashboard.putNumber("driveTrainPositionI", driveTrainPositionI);
    SmartDashboard.putNumber("driveTrainPositionD", driveTrainPositionD);
    setFieldRelativeAngle(180);
    m_imu.configFactoryDefault();
    resetRoll();
  }

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      MotorCanIdConstants.kDriveFrontLeft,
      MotorCanIdConstants.kSwerveFrontLeft,
      DriveTrainConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      MotorCanIdConstants.kDriveFrontRight,
      MotorCanIdConstants.kSwerveFrontRight,
      DriveTrainConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      MotorCanIdConstants.kDriveRearLeft,
      MotorCanIdConstants.kSwerveRearLeft,
      DriveTrainConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      MotorCanIdConstants.kDriveRearRight,
      MotorCanIdConstants.kSwerveRearRight,
      DriveTrainConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 m_imu = new Pigeon2(ControlSytemCanIdConstants.kPigeonId);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveTrainConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  

  public boolean autoBalanceOnPlatform() {
    // Ballance on platform at slight angle so wheels hit one at a time
    angleTargetDegrees = 10;
    double balanceSpeed = ballancePID.calculate(getRoll(), 0);
    driveOrientationClampedXY(balanceSpeed, 0, angleTargetDegrees, 0.4);
    if (AutoUtils.aproxEqual(getRoll(), 0, 1)) {
      return true;
    }
    return false;
  }

  public void autoNothing(){
    drive(0, 0, 0, true);
  }

  
  @Override
  public void periodic() {

    if (RobotState.isDisabled()) {
      angleTargetDegrees = getYaw();
      timeLastDisabled = Timer.getFPGATimestamp();
    }

    m_odometry.update(
        Rotation2d.fromDegrees(getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Update the odometry in the periodic block
    SmartDashboard.putNumber("getYaw", getYaw());
    SmartDashboard.putNumber("getPitch", getPitch());
    SmartDashboard.putNumber("getRoll", getRoll());
    SmartDashboard.putNumber("m_frontLeft.getWheelEncoder()", m_frontLeft.getWheelEncoder());

    driveTrainOrientationP = SmartDashboard.getNumber("driveTrainOrientationP", driveTrainOrientationP);
    driveTrainOrientationI = SmartDashboard.getNumber("driveTrainOrientationI", driveTrainOrientationI);
    driveTrainOrientationD = SmartDashboard.getNumber("driveTrainOrientationD", driveTrainOrientationD);
    orientationPID.setPID(driveTrainOrientationP, driveTrainOrientationI, driveTrainOrientationD);

    ballanceP = SmartDashboard.getNumber("ballanceP", ballanceP);
    ballanceI = SmartDashboard.getNumber("ballanceI", ballanceI);
    ballanceD = SmartDashboard.getNumber("ballanceD", ballanceD);
    ballancePID.setPID(ballanceP, ballanceI, ballanceD);

    driveTrainPositionP = SmartDashboard.getNumber("driveTrainPositionP", driveTrainPositionP);
    driveTrainPositionI = SmartDashboard.getNumber("driveTrainPositionI", driveTrainPositionI);
    driveTrainPositionD = SmartDashboard.getNumber("driveTrainPositionD", driveTrainPositionD);
    positionPID.setPID(driveTrainPositionP, driveTrainPositionI, driveTrainPositionD);

    maxDrivetrainMPS = SmartDashboard.getNumber("maxDrivetrainMPS",  maxDrivetrainMPS);


    // Heading reset mode
    if (m_driverController.getBackButton()) {
      angleTargetingMode = false;
      resetYawOffset(); // Yaw offset resets field oriented
      resetRoll(); // Roll and pitch are just here for convinience
      resetPitch();
    } else {
      // Robot centric mode when pressing either bumper
      if (m_driverController.getRightBumper() || m_driverController.getRightBumper()) {
        // Set target angle to current angle so robot maintains orientation when back in field centric
        angleTargetDegrees = getYaw();
        angleTargetingMode = false;
        fieldRelativeControlState = false;
      }
      // Field centric with heading control normally
      else {
        angleTargetingMode = true;
        fieldRelativeControlState = true;
      }
    }
    // Autobalance when start button pressed
    if (m_driverController.getStartButtonPressed()) {
      onPlatform = false;
      resetRoll();
    }

    // Drive ballance
    if (m_driverController.getStartButton()) {
      autoBalanceOnPlatform();
      return; // Jump out if auto balance
    }

    double controllerLeftX = -OperatorUtils.continous_deadband(m_driverController.getLeftX(),
        OperatorConstants.kDriveDeadband);
    double controllerLeftY = -OperatorUtils.continous_deadband(m_driverController.getLeftY(),
        OperatorConstants.kDriveDeadband);
    double controllerRightX = m_driverController.getRightX();
    double controllerRightY = m_driverController.getRightY();

    if (m_driverController.getLeftStickButton()) {
      controllerLeftX = controllerLeftX / 4;
      controllerLeftY = controllerLeftY / 4;
    }

    // Heading target driving
    if (angleTargetingMode)
    {
      // Only change target heading if steering stick is pressed far in any direction
      double radius = Math.sqrt(controllerRightX * controllerRightX + controllerRightY * controllerRightY);
      if (radius > 0.75)
        // Cartesian to polar.
        angleTargetDegrees = -(Math.atan2(controllerRightX, -controllerRightY) / Math.PI) * 180;

      driveOrientation(
          controllerLeftY,
          controllerLeftX,
          angleTargetDegrees);
    }
    // No heading target driving
    else
      drive(
          controllerLeftY,
          controllerLeftX,
          -controllerRightX,
          fieldRelativeControlState);

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public double getRotationLimited(double xSpeed, double ySpeed, double rotation) {

    double adjustedRotation;
    double rotMultiplier = 1;
    double totalMagnatudeXY = Math.abs(xSpeed) + Math.abs(ySpeed);

    // clamp rotation
    rotation = OperatorUtils.clampValue(rotation, 1);

    rotMultiplier = 1 - (totalMagnatudeXY / 2);
    adjustedRotation = rotMultiplier * rotation;
    return adjustedRotation;
  }

  // TODO: Should make the limit in mps
  public void driveOrientationClampedXY(double xSpeed, double ySpeed, double desiredOrentation, double xyLimit) {
    double clampedX = OperatorUtils.clampValue(xSpeed, xyLimit);
    double clampedY = OperatorUtils.clampValue(ySpeed, xyLimit);
    driveOrientation(clampedX, clampedY, desiredOrentation);
  }

  public void driveOrientation(double xSpeed, double ySpeed, double desiredOrentation) {
    SmartDashboard.putNumber("angleTargetDegrees", angleTargetDegrees);

    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeedCommanded = xSpeed;
    ySpeedCommanded = ySpeed;

    double rotationRequest = orientationPID.calculate(getYaw(), desiredOrentation);

    m_currentRotation = getRotationLimited(xSpeed, ySpeed, rotationRequest);

    if (xSpeedCommanded == 0 && ySpeedCommanded == 0 && m_currentRotation == 0) {
      setTurningFormation();
      return;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * maxDrivetrainMPS;
    double ySpeedDelivered = ySpeedCommanded * maxDrivetrainMPS;
    double rotDelivered = m_currentRotation * DriveTrainConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveTrainConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
            Rotation2d.fromDegrees(getYaw())));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxDrivetrainMPS);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeedCommanded = xSpeed;
    ySpeedCommanded = ySpeed;
    m_currentRotation = getRotationLimited(xSpeed, ySpeed, rot);

    if (xSpeedCommanded == 0 && ySpeedCommanded == 0 && m_currentRotation == 0) {
      setTurningFormation();
      return;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * maxDrivetrainMPS;
    double ySpeedDelivered = ySpeedCommanded * maxDrivetrainMPS;
    double rotDelivered = m_currentRotation * DriveTrainConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveTrainConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxDrivetrainMPS);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the wheels into an turning formation to prevent movement and be able to
   * quickly turn
   */
  public void setTurningFormation() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, maxDrivetrainMPS);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. 
   * 
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingFromStart() {
    return Rotation2d.fromDegrees(m_imu.getYaw()).getDegrees();
  }

  public double getYaw() {
    return Rotation2d.fromDegrees(m_imu.getYaw()).getDegrees() - yawOffset;
  }

  public double getRoll() {
    return Rotation2d.fromDegrees(m_imu.getRoll()).getDegrees() - rollOffset;
  }

  public void resetRoll() {
    rollOffset = Rotation2d.fromDegrees(m_imu.getRoll()).getDegrees();
  }

  public double getPitch() {
    return Rotation2d.fromDegrees(m_imu.getPitch()).getDegrees() - pitchOffset;
  }

  public void resetPitch() {
    pitchOffset = Rotation2d.fromDegrees(m_imu.getPitch()).getDegrees();
  }

  public void setYawOffset(double value, double currentAngle) {
    yawOffset = value;
    angleTargetDegrees = currentAngle;
  }

  public void resetYawOffset() {
    setYawOffset(getHeadingFromStart(), 0);
    fieldRelativeControlState = true;
  }

  public void setFieldRelativeAngle(double angle) {
    setYawOffset(getHeadingFromStart() + angle, angle);
    fieldRelativeControlState = true;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return m_imu.get * (DriveTrainConstants.kGyroReversed ? -1.0 : 1.0);
  // }
}