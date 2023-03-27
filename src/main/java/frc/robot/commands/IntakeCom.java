// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeCom extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_subsystem;
  private final BooleanSupplier Up, down, outward, inward;
  /**
   * Creates a new IntakeCom.
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCom(IntakeSubsystem subsystem,BooleanSupplier Upward, BooleanSupplier Downard, BooleanSupplier outside, BooleanSupplier inside) {
    m_subsystem = subsystem;
    Up = Upward;
    down = Downard;
    outward = outside;
    inward = inside;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.InMovement(Up.getAsBoolean(), down.getAsBoolean());
    m_subsystem.Pivoting(outward.getAsBoolean(), inward.getAsBoolean());
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
