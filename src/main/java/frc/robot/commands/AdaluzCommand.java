// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AdaluzSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class AdaluzCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AdaluzSubsystem m_AdaluzSubsystem;
    private final DoubleSupplier silly;
    private final DoubleSupplier mimis;
    private final DoubleSupplier turning;






  /**
   * Creates a new AdaluzCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AdaluzCommand(AdaluzSubsystem subsystem, DoubleSupplier forward1, DoubleSupplier back1, DoubleSupplier turn) {
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_AdaluzSubsystem = subsystem;
    silly = forward1;
    mimis = back1;
    turning = turn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AdaluzSubsystem.moveandturn(silly.getAsDouble() - mimis.getAsDouble(), turning.getAsDouble());
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