// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Hello git test 

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//Intake 2 Neos, Extender 2 Neos, Elevator 2 CIMS or talons
public class Elevatorsubsystem extends SubsystemBase {
  /** Creates a new Elevatorsubsystem. */

  TalonSRX ElevaM1 = new TalonSRX(Constants.ElevatorMotor1);
  TalonSRX ElevaM2 = new TalonSRX(Constants.ElevatorMotor2);

  public Elevatorsubsystem() {
    ElevaM1.configFactoryDefault();
    ElevaM2.configFactoryDefault();

    ElevaM1.setNeutralMode(NeutralMode.Brake);
    ElevaM2.setNeutralMode(NeutralMode.Brake);
    
    ElevaM2.follow(ElevaM1);
    ElevaM2.setInverted(InvertType.FollowMaster);
  }
  
  // public void ElevatorMovement(double power){
  //   ElevaM1.set(ControlMode.PercentOutput, power);
  // }
  public void Extendermoment(boolean in, boolean out){
    ElevaM1.set(ControlMode.PercentOutput, 0);
    if(out){
      ElevaM1.set(ControlMode.PercentOutput, -1);
    }
    if(in){
      ElevaM1.set(ControlMode.PercentOutput, 1);
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
