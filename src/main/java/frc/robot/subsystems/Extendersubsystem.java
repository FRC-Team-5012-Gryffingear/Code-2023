// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Hello git test 

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//Intake 2 Neos, Extender 2 Neos, Elevator 2 CIMS or talons
public class Extendersubsystem extends SubsystemBase {
    //Change if they are not brushless
    CANSparkMax NeoEx1 = new CANSparkMax(Constants.ElevatorMotor1, MotorType.kBrushless);
    CANSparkMax NeoEx2 = new CANSparkMax(Constants.ElevatorMotor2, MotorType.kBrushless);
  /** Creates a new Extendersubsystem. */
  public Extendersubsystem() {
    NeoEx1.setIdleMode(IdleMode.kBrake);
    NeoEx2.setIdleMode(IdleMode.kBrake);

    NeoEx2.follow(NeoEx1, true);
  }
  public void EnxtendingMovement(boolean out, boolean in){
    NeoEx1.set(0);
    if(in){
        NeoEx1.set(-1);
    }
    if(out){
        NeoEx1.set(1);
    }
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
