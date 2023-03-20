// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class Autos extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final SwerveSubsystem swerve;
  private Timer times = new Timer();
  private double target = 0;
  private PIDController controller = new PIDController(0.02, 0, 0.02);


  /**
   * Creates a new Autos.
   * @param subsystem The subsystem used by this command.
   */
  public Autos(SwerveSubsystem subsystem) {
    swerve = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.zeroGyro();
    times.reset();
    times.start();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  //is it alligned
  public void execute() {
    swerve.Yaw();
    double percent = swerve.Yaw() / 360;
    //swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerve.getGyro()));
  if(times.get() > 0.5){
   swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(percent, 0, 0, swerve.getGyro()));
   if(times.get() > 4){
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerve.getGyro()));
   }
  }
 
    //   swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-1,0,0, swerve.getGyro()));
    // while(times.get() > 0.5){
    //   if(!swerve.YawDetectL()){
    //     swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0.25, swerve.getGyro()));
    //   } else if(!swerve.YawDetectR()){
    //     swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,-0.25, swerve.getGyro()));
    //   }
    // }

      //   if(!swerve.PitchDetectF()){
      //     swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.25,0,0, swerve.getGyro()));
      //   } else if(!swerve.PitchDetectB()){
      //     swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.25,0,0, swerve.getGyro()));
      //   } else if(swerve.PitchDetectB() && swerve.PitchDetectF()){
      //     swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, swerve.getGyro()));
      //   }
  
      // }
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
