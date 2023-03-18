// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class AutoComs extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final SwerveSubsystem swerve;
  private Timer quick = new Timer();
  /**
   * Creates a new AutoComs.
   * @param subsystem The subsystem used by this command.
   */
  public AutoComs(SwerveSubsystem subsystem) {
    swerve = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.zeroGyro();
    quick.reset();
    quick.start();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if(quick.get() > 0.5){
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-3, 0, 0, swerve.getGyro()));
        if(quick.get() > 5){
          swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerve.getGyro()));
        }
    //First 
      // if (quick.get() > 1.5){
      //   swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-1,0,0, swerve.getGyro()));
      // }
      // if(quick.get() > 5.5){
      //   swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, swerve.getGyro()));
      // }
    //Second
  //     if(quick.get() > 1.75){
  //       swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-2,0,0 ,swerve.getGyro()));
  //     }
  //     if(quick.get() > 3.75){
  //       swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(1,0,0, swerve.getGyro()));
  //     }
  //     if(quick.get() > 4.25){
  //       swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, swerve.getGyro()));
  //     }
  // }
  }
  
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
