// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Hola testing 2

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax intake = new CANSparkMax(Constants.SparkIntake, MotorType.kBrushless);
  VictorSPX pivot = new VictorSPX(Constants.pivotIntake);
  public IntakeSubsystem() {
    pivot.configFactoryDefault();
    intake.restoreFactoryDefaults();

    pivot.setNeutralMode(NeutralMode.Brake);
    intake.setIdleMode(IdleMode.kBrake);
    //add if inverted later
  }
  public void InMovement(boolean power, boolean back){
    intake.set(0);
    if(power){
        intake.set(0.25);
    }
    if(back){
        intake.set(-0.25);
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
