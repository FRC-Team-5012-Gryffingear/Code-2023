// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.swerveCommand;
import frc.robot.otherInfo.controllerConstant;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final Joystick driver = new Joystick(Constants.driverController);
 // private final Joystick operator = new Joystick(Constants.operatorController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(new swerveCommand(swerve,
    () -> -modifyAxis(driver.getRawAxis(controllerConstant.LEFT_STICK_X)) * swerve.Max_Velocity,
    () -> -modifyAxis(driver.getRawAxis(controllerConstant.LEFT_STICK_Y)) * swerve.Max_Velocity,
    () -> -modifyAxis(driver.getRawAxis(controllerConstant.RIGHT_STICK_X)) * swerve.Max_Angle,
    () -> driver.getRawButton(controllerConstant.B)));
    // Configure the trigger bindings
    
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
    return (command);
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
