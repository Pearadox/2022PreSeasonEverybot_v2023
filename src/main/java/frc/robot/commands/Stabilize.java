// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Stabilize extends CommandBase {

  private DriveTrain _driveTrain;
  private ADIS16470_IMU _gyro;

  private final double _Kp = 0.4;
  private final double _Ki = 0.15;
  private final double _Kd = 0;
  private PIDController _pid = new PIDController(_Kp, _Ki, _Kd);

  /** Creates a new Stabilize. */
  public Stabilize(DriveTrain driveTrain, ADIS16470_IMU gyro) {
    _driveTrain = driveTrain;
    _gyro = gyro;
    addRequirements(_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    double error = _gyro.getAngle();
    SmartDashboard.putNumber("Error", error);
    double pidOut = _pid.calculate(error, 0);
    SmartDashboard.putNumber("PID Out", pidOut);

    double drivePower = pidOut/15;
    SmartDashboard.putNumber("Drive Power", drivePower);
    if (Math.abs(drivePower) > 0.5) {
      drivePower = Math.copySign(0.5, drivePower);
    }
    if (Math.abs(drivePower) < 0.22 && (Math.abs(pidOut)) < 6) {
      drivePower = Math.copySign(0.22, drivePower);
    }
    if(!(-2 < error && error < 2)) {
      _driveTrain.drive(0, drivePower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}