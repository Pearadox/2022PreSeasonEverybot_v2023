// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  private final Joystick _controller = new Joystick(0);
  private final ADIS16470_IMU _gyro = new ADIS16470_IMU();
  private final DriveTrain _driveTrain = new DriveTrain(_gyro);
  private final Intake _intake = new Intake();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    _gyro.reset();
    _gyro.setYawAxis(IMUAxis.kZ);
    configureButtonBindings();
    _driveTrain.setDefaultCommand(new ArcadeDrive(_driveTrain, _controller));
    _intake.setDefaultCommand(new IntakeStop(_intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(_controller, JoystickConstants.BUMPER_RIGHT).whileTrue(new IntakeIn(_intake));
    new JoystickButton(_controller, JoystickConstants.BUMPER_LEFT).whileTrue(new IntakeOut(_intake));
    new JoystickButton(_controller, JoystickConstants.Y).whileTrue(new RunCommand(_intake::raise, _intake));
    new JoystickButton(_controller, JoystickConstants.A).whileTrue(new RunCommand(_intake::lower, _intake));
    new JoystickButton(_controller, JoystickConstants.B).whileTrue(new Stabilize(_driveTrain, _gyro));
    new JoystickButton(_controller, JoystickConstants.X).onTrue(
      new InstantCommand(_gyro::reset).andThen(new InstantCommand(_driveTrain::resetEncoders)));
    new JoystickButton(_controller, JoystickConstants.LOGO_RIGHT).onTrue(new EngageCS(_driveTrain, _gyro));
    new JoystickButton(_controller, JoystickConstants.LOGO_LEFT).onTrue(new AutoGetSphubeAndScore(_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Trajectory exampleTrajectory = PathPlanner.loadPath("sphube pickup", new
    // PathConstraints(3, 2),true);
    // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("sphube pickup", true, new PathConstraints(4, 3));
    // // This is just an example event map. It would be better to have a constant,
    // // global event map
    // // in your code that will be used by all path following commands.
    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("IntakeDown", new AutoIntakeDown(_intake));
    // eventMap.put("IntakeIn", new AutoIntakeIn(_intake));
    // eventMap.put("IntakeStop", new IntakeStop(_intake));
    // eventMap.put("IntakeOut", new AutoIntakeOut(_intake));

    // // Create the AutoBuilder. This only needs to be created once when robot code
    // // starts, not every time you want to create an auto command. A good place to
    // // put this is in RobotContainer along with your subsystems.
    // RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    //     _driveTrain::getPose, // Pose2d supplier
    //     _driveTrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    //     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //     DriveTrainConstants.kDriveKinematics, // SwerveDriveKinematics
    //     new SimpleMotorFeedforward(
    //         DriveTrainConstants.ksVolts,
    //         DriveTrainConstants.kvVoltSecondsPerMeter,
    //         DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
    //     _driveTrain::getWheelSpeeds,
    //     new PIDConstants(DriveTrainConstants.kPDriveVel, 0, 0),
    //     _driveTrain::tankDriveVolts,
    //     eventMap,
    //     true, // Should the path be automatically mirrored depending on alliance color.
    //           // Optional, defaults to true
    //     _driveTrain // The drive subsystem. Used to properly set the requirements of path following
    //                 // commands

    // );
    // return autoBuilder.fullAuto(pathGroup);
    // Trajectory exampleTrajectory = PathPlanner.loadPath("TestForwardv1", new PathConstraints(2, 1));
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveTrainConstants.ksVolts,
                DriveTrainConstants.kvVoltSecondsPerMeter,
                DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
            DriveTrainConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                0.2,
                0.1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveTrainConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    _driveTrain.resetEncoders();
    _driveTrain.zeroHeading();
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        _driveTrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveTrainConstants.ksVolts,
            DriveTrainConstants.kvVoltSecondsPerMeter,
            DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrainConstants.kDriveKinematics,
        _driveTrain::getWheelSpeeds,
        new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
        new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
        _driveTrain::tankDriveVolts,
        _driveTrain);
    _driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
    return ramseteCommand;
  }
}
