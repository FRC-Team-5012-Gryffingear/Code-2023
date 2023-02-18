// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intakesubsystem;

import java.lang.module.ModuleDescriptor.Opens;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.IntegerArrayEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
    private final Intakesubsystem Intakesub;
    private final BooleanSupplier Cube, Cone, Open;
  /**
   * Creates a new IntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(Intakesubsystem subsystem, BooleanSupplier Cubes, BooleanSupplier Cones, BooleanSupplier Opens) {
    Intakesub = subsystem;
    Cube = Cubes;
    Cone = Cones;
    Open = Opens;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intakesub.IntakeMovement(Cube.getAsBoolean(), Cone.getAsBoolean(), Open.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
