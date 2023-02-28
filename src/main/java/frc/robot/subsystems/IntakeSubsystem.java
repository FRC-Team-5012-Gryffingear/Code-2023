// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Hola testing 2

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax NeoIntake1 = new CANSparkMax(Constants.NeoIntake1, MotorType.kBrushless);
  CANSparkMax NeoIntake2 = new CANSparkMax(Constants.NeoIntake2, MotorType.kBrushless);

  public IntakeSubsystem() {
    NeoIntake1.setIdleMode(IdleMode.kBrake);
    NeoIntake2.setIdleMode(IdleMode.kBrake);

    NeoIntake2.setInverted(true);
    NeoIntake2.follow(NeoIntake1);
  }

  public void Intake(boolean in, boolean out){
    NeoIntake1.set(0);
    if(out){
        NeoIntake1.set(1);
    }
    if(in){
        NeoIntake1.set(-1);
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
