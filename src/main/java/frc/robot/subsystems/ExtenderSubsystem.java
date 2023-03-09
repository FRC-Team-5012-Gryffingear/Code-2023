// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Hola testing 2

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  /** Creates a new ExtenderSubsystem. */
  TalonSRX CIM1 = new TalonSRX(Constants.Extender1);
  TalonSRX CIM2 = new TalonSRX(Constants.Extender2);

  public ExtenderSubsystem() {
    CIM1.configFactoryDefault();
    CIM2.configFactoryDefault();

    CIM1.setNeutralMode(NeutralMode.Brake);
    CIM2.setNeutralMode(NeutralMode.Brake);

    //CIM2.setInverted(InvertType.InvertMotorOutput);
    CIM2.follow(CIM1,FollowerType.PercentOutput);

    //wait to see if needed to invert
  }

  public void Extending(boolean in, boolean out){
    CIM1.set(ControlMode.PercentOutput, 0);
    if(out){
        CIM1.set(ControlMode.PercentOutput, 0.5);
    }
    if(in){
        CIM1.set(ControlMode.PercentOutput, -.5);
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
