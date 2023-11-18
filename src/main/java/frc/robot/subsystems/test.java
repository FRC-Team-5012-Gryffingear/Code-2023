// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class test extends SubsystemBase {
  /** Creates a new test. */
  TalonSRX talon1 = new TalonSRX(0);
  TalonFX talon2 = new TalonFX(1);
  VictorSPX talon3 = new VictorSPX(2);

  public test() {
    talon1.configFactoryDefault();
    talon1.setNeutralMode(NeutralMode.Coast);
    talon1.setInverted(InvertType.InvertMotorOutput);

    talon2.configFactoryDefault();
    talon2.setNeutralMode(NeutralMode.Coast);

    talon3.configFactoryDefault();
    talon3.setNeutralMode(NeutralMode.Coast);
  }

  public void monkey(double turn, double forward){
    talon3.set(ControlMode.PercentOutput, forward-turn);
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
