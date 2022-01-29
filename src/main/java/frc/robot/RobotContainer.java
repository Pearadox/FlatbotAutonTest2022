// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FiveBallAuton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.util.TrajectoryCache;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static frc.robot.Constants.*;

import java.nio.file.Paths;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static Drivetrain drivetrain = Drivetrain.getInstance();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final Joystick driverJoystick = new Joystick(0);
  public final XboxController driverXbox = new XboxController(2);
  public SendableChooser<String> pathSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    pathSelector = new SendableChooser<>();
    SmartDashboard.putData(pathSelector);
    loadTrajectoryPaths();
    drivetrain.setDefaultCommand(new RunCommand(
      () -> drivetrain.arcadeDrive(-driverXbox.getRawAxis(1), driverXbox.getRawAxis(4), true)
    , drivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public void loadTrajectoryPaths() {
    TrajectoryCache.clear();
    // sendCacheTrajectory("Slalom", "output/SlalomPath");
    // pathSelector.setDefaultOption("ThreeBallAuton", "ThreeBallAuton");
    // pathSelector.addOption("SixBallBackAuton", "SixBallBackAuton");
    pathSelector.addOption("Straight", "Straight");
    pathSelector.addOption("3 Ball Test", "3 Ball Test");
    pathSelector.addOption("HalfArc", "HalfArc");
    pathSelector.addOption("BackHalfArc", "BackHalfArc");
    pathSelector.addOption("TerminalLoad", "TerminalLoad");
    pathSelector.addOption("5 Ball Auton", "5 Ball Auton");

    TrajectoryCache.add("3 Ball Test", "3 Ball Test");
    TrajectoryCache.add("Straight", "Straight");
    TrajectoryCache.add("HalfArc", "HalfArc");
    TrajectoryCache.add("BackHalfArc", "BackHalfArc");
    TrajectoryCache.add("TerminalLoad", "TerminalLoad");
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DrivetrainConstants.kA, DrivetrainConstants.kV, DrivetrainConstants.kA),
        DrivetrainConstants.KINEMATICS,
        10
    );

    var TrajectoryConfig = new TrajectoryConfig(
      DrivetrainConstants.MAX_VELOCITY, DrivetrainConstants.MAX_ACCEL);
    if (pathSelector.getSelected().equals("5 Ball Auton")) {
      return new FiveBallAuton(drivetrain).andThen(() -> drivetrain.setVoltages(0, 0));
    }
    Trajectory pathTrajectory = TrajectoryCache.get(pathSelector.getSelected());
    RamseteCommand ramseteCommand = createRamseteCommand(pathTrajectory);
    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(pathTrajectory.getInitialPose());
    // drivetrain.gyro.setAngleAdjustment(pathTrajectory.getInitialPose().getRotation().getRadians() - drivetrain.gyro.getYaw());
    // drivetrain.gyro.zeroYaw();
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.setVoltages(0, 0));
  }

  public RamseteCommand createRamseteCommand(Trajectory pathTrajectory) {
    return new RamseteCommand(
      pathTrajectory,
      drivetrain::getPose,
      new RamseteController(DrivetrainConstants.B, DrivetrainConstants.ZETA),
      new SimpleMotorFeedforward(DrivetrainConstants.kS,
                                 DrivetrainConstants.kV,
                                 DrivetrainConstants.kA),
      DrivetrainConstants.KINEMATICS,
      drivetrain::getWheelSpeeds,
      new PIDController(0, 0, 0),
      new PIDController(0, 0, 0),
      // RamseteCommand passes volts to the callback
      drivetrain::tankDriveVolts,
      drivetrain
    );
  }
}
