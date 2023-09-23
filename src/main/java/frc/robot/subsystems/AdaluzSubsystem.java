// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AdaluzSubsystem extends SubsystemBase {
  /** Creates a new AdaluzSubsytem. */
    TalonSRX sunrise = new TalonSRX(Constants.talon1);
    TalonSRX worlds = new TalonSRX(Constants.talon2);
    TalonSRX trophy = new TalonSRX(Constants.talon3);
    TalonSRX nice = new TalonSRX(Constants.talon4);


  public AdaluzSubsystem() {
    sunrise.configFactoryDefault();
    worlds.configFactoryDefault();
    trophy.configFactoryDefault();
    nice.configFactoryDefault();

    sunrise.setNeutralMode(NeutralMode.Coast);
    worlds.setNeutralMode(NeutralMode.Coast);
    trophy.setNeutralMode(NeutralMode.Coast);
    nice.setNeutralMode(NeutralMode.Coast);

    sunrise.follow(worlds);
    nice.follow(trophy);

    worlds.setInverted(InvertType.InvertMotorOutput);
    trophy.setInverted(InvertType.InvertMotorOutput);
  }

  public void moveandturn(double Power, double turn) {
    worlds.set(ControlMode.PercentOutput, Power - turn);
    trophy.set(ControlMode.PercentOutput, Power + turn);
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