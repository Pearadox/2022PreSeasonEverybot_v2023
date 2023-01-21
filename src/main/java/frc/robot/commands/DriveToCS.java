// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToCS extends CommandBase {

  private DriveTrain _driveTrain;
  private RelativeEncoder _encoder;

  /** Creates a new DriveToCS. */
  public DriveToCS(DriveTrain driveTrain) {
    _driveTrain = driveTrain;
    _encoder = _driveTrain.getEncoder();
    addRequirements(_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _encoder.setPosition(0);
    double circumference = Units.inchesToMeters(Math.PI * 6.0);
    double gearReduction = 10.71;
    _encoder.setPositionConversionFactor(circumference / gearReduction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _driveTrain.drive(0, 0.65);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double position = -1 * _encoder.getPosition();
    SmartDashboard.putNumber("Encoder Position", position);
    if (position <= -1.8) return true;

    return false;
  }
}
