// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private VictorSPX _intake = new VictorSPX(20);
  private CANSparkMax _arm = new CANSparkMax(21, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    _intake.configFactoryDefault();
    _arm.restoreFactoryDefaults();
    _arm.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void in() {
    _intake.set(ControlMode.PercentOutput, -1);
  }

  public void out() {
    _intake.set(ControlMode.PercentOutput, 1);
  }

  public void stopIntake() {
    _intake.set(ControlMode.PercentOutput, 0.00);
  }


  public void lower() {
    _arm.set(-.1);
  }

  public void raise() {
    _arm.set(.1);
  }

  public void stopArm() {
    _arm.set(0);
  }

  public void stop() {
    stopIntake();
    stopArm();
  }
}
