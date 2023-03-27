// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Hola testing 2

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  VictorSPX victored = new VictorSPX(Constants.VictorIntake);
  VictorSPX pivot = new VictorSPX(Constants.pivotIntake);
  public IntakeSubsystem() {
    victored.configFactoryDefault();
    pivot.configFactoryDefault();

    pivot.setNeutralMode(NeutralMode.Brake);
    victored.setNeutralMode(NeutralMode.Brake);
    //add if inverted later
  }
  public void InMovement(boolean power, boolean back){
    victored.set(ControlMode.PercentOutput, 0);
    if(power){
        victored.set(ControlMode.PercentOutput, 0.25);
    }
    if(back){
        victored.set(ControlMode.PercentOutput, -0.25);
    }
  }
//RB outward
  public void Pivoting(boolean outward, boolean inward){
    pivot.set(ControlMode.PercentOutput, 0);
    if(outward){
        pivot.set(ControlMode.PercentOutput, 0.25);
    }
    if(inward){
        pivot.set(ControlMode.PercentOutput, -0.25);
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