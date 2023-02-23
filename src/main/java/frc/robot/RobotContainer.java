// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


//import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtenderCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.otherInfo.controllerConstant;
import frc.robot.subsystems.Elevatorsubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Extendersubsystem;
import frc.robot.subsystems.Intakesubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand example = new ExampleCommand(m_exampleSubsystem);
  
  private final Intakesubsystem Intakesubsys = new Intakesubsystem();

  private final Extendersubsystem Extendsubsys = new Extendersubsystem();

  private final Elevatorsubsystem ElevSubsys = new Elevatorsubsystem();

  private final Joystick driverController = new Joystick(Constants.DriverController);
  private final Joystick operatorController = new Joystick(Constants.OperatorController);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
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
    ElevSubsys.setDefaultCommand(new ElevatorCommand(ElevSubsys, 
    () -> operatorController.getRawAxis(controllerConstant.RIGHT_TRIGGER), 
    () -> operatorController.getRawAxis(controllerConstant.LEFT_TRIGGER)));

    Intakesubsys.setDefaultCommand(new IntakeCommand(Intakesubsys, 
    () -> operatorController.getRawButton(controllerConstant.B),
    () -> operatorController.getRawButton(controllerConstant.A)));

    Extendsubsys.setDefaultCommand(new ExtenderCommand(Extendsubsys,
    () -> operatorController.getRawButton(controllerConstant.RB), 
    () -> operatorController.getRawButton(controllerConstant.LB)));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
      //  .onTrue(new ExampleCommand(m_exampleSubsystem));

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
    return example;
  }
}
