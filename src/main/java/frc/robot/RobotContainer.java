// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.AutoComs;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtenderCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.swerveCommand;
import frc.robot.otherInfo.controllerConstant;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand command = new ExampleCommand(m_exampleSubsystem);


    // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final IntakeSubsystem intakesubsys = new IntakeSubsystem();
  private final ExtenderSubsystem extendo = new ExtenderSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  private final AutoComs comman = new AutoComs(intakesubsys);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final Joystick driver = new Joystick(Constants.driverController);
  private final Joystick operator = new Joystick(Constants.operatorController);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // swerve.setDefaultCommand(new swerveCommand(swerve,
    // () -> -modifyAxis(driver.getRawAxis(controllerConstant.LEFT_STICK_X)) * swerve.Max_Velocity,
    // () -> -modifyAxis(driver.getRawAxis(controllerConstant.LEFT_STICK_Y)) * swerve.Max_Velocity,
    // () -> -modifyAxis(driver.getRawAxis(controllerConstant.RIGHT_STICK_X)) * swerve.Max_Angle,
    // () -> driver.getRawButton(controllerConstant.B)));
    // // Configure the trigger bindings
    swerve.setDefaultCommand(new swerveCommand(swerve, 
    () -> modifyAxis(driver.getRawAxis(controllerConstant.RIGHT_STICK_Y)) * swerve.Max_Velocity,
    () -> modifyAxis(driver.getRawAxis(controllerConstant.RIGHT_STICK_X)) * swerve.Max_Velocity,
    () -> modifyAxis(driver.getRawAxis(controllerConstant.LEFT_STICK_X)) * swerve.Max_Angle,
    () -> driver.getRawButton(controllerConstant.A)));
    
    elevator.setDefaultCommand(new ElevatorCommand(elevator, 
    () -> operator.getRawAxis(controllerConstant.RIGHT_TRIGGER), 
    () -> operator.getRawAxis(controllerConstant.LEFT_TRIGGER)));

    intakesubsys.setDefaultCommand(new IntakeCommand(intakesubsys, 
    () -> operator.getRawButton(controllerConstant.Y), 
    () -> operator.getRawButton(controllerConstant.X)));

    extendo.setDefaultCommand(new ExtenderCommand(extendo, 
    () -> operator.getRawButton(controllerConstant.RB), 
    () -> operator.getRawButton(controllerConstant.LB)));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
// return (comman);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(swerve.Max_Velocity, 3).setKinematics(swerve.SwerveKinematics);
    //Trajectory of bot
    Trajectory trajects = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)),
     List.of(
      new Translation2d(1, 0),
      new Translation2d(1,-1)
     ), new Pose2d(2,-1, Rotation2d.fromDegrees(180)),
     trajectoryConfig);

     SwerveControllerCommand swerveControllerCommands = new SwerveControllerCommand(trajects, swerve::getPose, swerve.SwerveKinematics, null, swerve::setModuleStates, swerve);


    return new SequentialCommandGroup(new InstantCommand(() -> swerve.resetOdometry(trajects.getInitialPose())),
    swerveControllerCommands, 
    new InstantCommand(() -> swerve.drive(new ChassisSpeeds(0,0,0))));
  }


  private static double deadband(double val, double deadband){
    if(Math.abs(val) > deadband){
      if(val > 0){
        return (val - deadband) / (1.0 - deadband);
      } else{
        return (val + deadband) / (1.0 - deadband);
      } 
    } else{
      return 0.0;
    }
  }

  private static double modifyAxis(double value){
    //deadband
    value = deadband(value,0.05);
    //squared axis
    value = Math.copySign(value * value, value);
    
    return value;
  }
}
