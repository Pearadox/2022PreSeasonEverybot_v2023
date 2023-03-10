// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutoIntakeOut extends CommandBase {

  private Intake _intake;
  private long _start;

  public AutoIntakeOut(Intake intake) {
    _intake = intake;
    addRequirements(_intake);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _start = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intake.out();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(System.currentTimeMillis() - _start < 750){
      return false;
    }
    return true;
  }
}
