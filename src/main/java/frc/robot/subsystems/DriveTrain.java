// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax _frontLeft = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax _backLeft = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax _frontRight = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax _backRight = new CANSparkMax(10, MotorType.kBrushless);
  private DifferentialDrive _drive = new DifferentialDrive(_frontLeft, _frontRight);
  private ADIS16470_IMU _gyro;
  private DifferentialDriveOdometry _odometry;
  private RelativeEncoder _leftEncoder;
  private RelativeEncoder _rightEncoder;
  private Pose2d _pose;
  
  public DriveTrain(ADIS16470_IMU gyro) {
    SendableRegistry.add(_drive, "drive");

    _frontLeft.restoreFactoryDefaults();
    _backLeft.restoreFactoryDefaults();
    _frontRight.restoreFactoryDefaults();
    _backRight.restoreFactoryDefaults();

    _backLeft.follow(_frontLeft);
    _backRight.follow(_frontRight);

    _frontLeft.burnFlash();
    _backLeft.burnFlash();
    _frontRight.burnFlash();
    _backRight.burnFlash();

    _gyro = gyro;

    _leftEncoder = _frontLeft.getEncoder();
    _rightEncoder = _frontRight.getEncoder();

    _leftEncoder.setPositionConversionFactor(DriveTrainConstants.kDistancePerWheelRevolutionMeters/DriveTrainConstants.kGearReduction);
    _rightEncoder.setPositionConversionFactor(DriveTrainConstants.kDistancePerWheelRevolutionMeters/DriveTrainConstants.kGearReduction);

    resetEncoders();
    _odometry =
         new DifferentialDriveOdometry(
          Rotation2d.fromDegrees(_gyro.getAngle()), _leftEncoder.getPosition(), _rightEncoder.getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", _gyro.getAngle());
    SmartDashboard.putNumber("Left Encoder", _leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", _rightEncoder.getPosition());
    _pose = _odometry.update(Rotation2d.fromDegrees(_gyro.getAngle()), _leftEncoder.getPosition(), _rightEncoder.getPosition());
      Pose2d pose = getPose();
      SmartDashboard.putNumber("Pose X", pose.getX());
      SmartDashboard.putNumber("Pose y", pose.getY());
  }
  
  public void teleopDrive(Joystick controller) {
    double axis4 = controller.getRawAxis(4);
    double axis1 = controller.getRawAxis(1);
    drive(axis4, axis1);
  }

  public void drive(double rotation, double direction){
    _drive.arcadeDrive(rotation, direction);      
  }

  public RelativeEncoder getEncoder() {
    return _frontLeft.getEncoder();
  }
   /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // return _pose;
    return _odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_leftEncoder.getVelocity() / 60, _rightEncoder.getVelocity() / 60);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _odometry.resetPosition(Rotation2d.fromDegrees(_gyro.getAngle()), _leftEncoder.getPosition(), _rightEncoder.getPosition(), pose);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _frontLeft.setVoltage(leftVolts);
    _backLeft.setVoltage(leftVolts);
    _frontRight.setVoltage(rightVolts);
    _backRight.setVoltage(rightVolts);
    
    _drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    _leftEncoder.setPosition(0);
    _rightEncoder.setPosition(0);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    _gyro.reset();
  }

}
