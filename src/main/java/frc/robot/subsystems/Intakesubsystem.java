// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Hello git test 

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intakesubsystem extends SubsystemBase {
    CANSparkMax Neo1 = new CANSparkMax(Constants.NeoIntake1, MotorType.kBrushless);
    CANSparkMax Neo2 = new CANSparkMax(Constants.NeoIntake2, MotorType.kBrushless);
    //Change if it is not brushless or else it will break motor

  /** Creates a new Intakesubsystem. */
  public Intakesubsystem() {
    //Change the settings of the CANSparks on the big computer
    Neo1.setIdleMode(IdleMode.kBrake);
    Neo2.setIdleMode(IdleMode.kBrake);

    Neo2.follow(Neo1);
    Neo2.setInverted(true);
  }

  //Intake 2 Neos, Extender 2 Neos, Elevator 2 CIMS or talons
  //Cube button = A, Cone button = B, Open button = X/
  /*
  public void IntakeMovement(boolean cube, boolean cone, boolean Open){
    Neo1.set(0);

    if(cube){
        Neo1.set(1);
        if(Neo1.getOutputCurrent() > 20){
            Neo1.set(0);
        }
    }
    if(cone){
        Neo1.set(1);
        if(Neo1.getOutputCurrent() > 20){
            Neo1.set(0);
        }
    }
    if(Open){
        Neo1.set(-1);
    }
    SmartDashboard.putNumber("Neo Strain", Neo1.getOutputCurrent());
    SmartDashboard.putNumber("Neo2 Strain", Neo2.getOutputCurrent());
  }
   * 
   */
  
/*INCASE doesnt work we switch controls to Open A and Close B
*/
  public void IntakeMovement(boolean open, boolean close){
   Neo1.set(0);
   if(open){
     Neo1.set(1);
   }
   if(close){
     Neo1.set(-1);
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
