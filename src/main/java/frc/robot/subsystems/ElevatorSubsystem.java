// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Hola testing 2

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  CANSparkMax NeoElev1 = new CANSparkMax(Constants.NeoElevator1, MotorType.kBrushless);
  CANSparkMax NeoElev2 = new CANSparkMax(Constants.NeoElevator2, MotorType.kBrushless);

  public ElevatorSubsystem() {
    NeoElev1.setIdleMode(IdleMode.kBrake);
    NeoElev2.setIdleMode(IdleMode.kBrake);

    NeoElev2.follow(NeoElev1, false);
  }

  public void Movement(double power){
    NeoElev1.set(power/2);
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
