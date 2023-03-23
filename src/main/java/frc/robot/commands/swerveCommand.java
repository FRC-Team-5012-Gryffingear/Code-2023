// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class swerveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final SwerveSubsystem swerve; 
  private final DoubleSupplier Xsupply;
  private final DoubleSupplier Ysupply;
  public final DoubleSupplier rotationSupply;
  
  private final BooleanSupplier zero, return0, backwardYaw;
  // JOJO REFERENCE OMG !1!!!111
  /**
   * Creates a new swerveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public swerveCommand(SwerveSubsystem subsystem, DoubleSupplier Xmaker, DoubleSupplier Ymaker, DoubleSupplier rotateMaker, BooleanSupplier button, BooleanSupplier Back0, BooleanSupplier Back180) {
    swerve = subsystem;
    Xsupply = Xmaker;
    Ysupply = Ymaker;
    zero = button;
    rotationSupply = rotateMaker;
    return0 = Back0;
    backwardYaw = Back180;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double percent = swerve.Yaw()/18;
    double percent180 = (180 - Math.abs(swerve.Yaw()))/18;
    SmartDashboard.putNumber("percent180", percent180);

    if(return0.getAsBoolean()){
      swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(Xsupply.getAsDouble(), Ysupply.getAsDouble(),percent,swerve.getGyro()));
    }else if(backwardYaw.getAsBoolean()){
      swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(Xsupply.getAsDouble(), Ysupply.getAsDouble(), percent180, swerve.getGyro()));
    }
    else{
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        Xsupply.getAsDouble(), 
        Ysupply.getAsDouble(), 
        rotationSupply.getAsDouble(), 
        swerve.getGyro()));
    }
    swerve.zeroManual(zero.getAsBoolean());
    //Constants.Front_RightSTEER_Offset = -Math.toRadians(324);
    //Constants.Back_LeftSTEER_Offset = -Math.toRadians(40.5);
     // Encoder Ports
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

