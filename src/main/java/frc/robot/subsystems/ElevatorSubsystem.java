// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Hola testing 2

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  TalonSRX elevMotor1 = new TalonSRX(Constants.Elevator1);
  TalonSRX elevMotor2 = new TalonSRX(Constants.Elevator2);

  public ElevatorSubsystem() {
    elevMotor1.configFactoryDefault();
    elevMotor2.configFactoryDefault();

    elevMotor1.setNeutralMode(NeutralMode.Brake);
    elevMotor2.setNeutralMode(NeutralMode.Brake);

    elevMotor2.follow(elevMotor1);
  }
  public void elevmovement(double power) {
    elevMotor1.set(ControlMode.PercentOutput, power);
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
