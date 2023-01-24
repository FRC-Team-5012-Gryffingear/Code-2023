// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class swerveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final SwerveSubsystem swerve;

  private final DoubleSupplier Xsupply;
  private final DoubleSupplier Ysupply;
  private final DoubleSupplier rotationSupply;

  private final BooleanSupplier zero;

  /**
   * Creates a new swerveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public swerveCommand(SwerveSubsystem subsystem, DoubleSupplier Xmaker, DoubleSupplier Ymaker, DoubleSupplier rotateMaker, BooleanSupplier button) {
    swerve = subsystem;
    Xsupply = Xmaker;
    Ysupply = Ymaker;
    zero = button;
    rotationSupply = rotateMaker;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        Xsupply.getAsDouble(), 
        Ysupply.getAsDouble(), 
        rotationSupply.getAsDouble(), 
        swerve.getGyro()));
    swerve.zeroGyro(zero.getAsBoolean());
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0.0,0.0,0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
