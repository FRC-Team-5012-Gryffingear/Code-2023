// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

  public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  // distance between wheels Left to right 
  PigeonIMU pigeon = new PigeonIMU(Constants.pigeonID);


  CANCoder test1 = new CANCoder(Constants.Front_LeftSTEER_Encoder);
  CANCoder test2 = new CANCoder(Constants.Front_RightSTEER_Encoder);
  CANCoder test3 = new CANCoder(Constants.Back_LeftSTEER_Encoder);
  CANCoder test4 = new CANCoder(Constants.Back_RightSTEER_Encoder);

  public static final double Voltage = 12.0;

  public static final double Max_Velocity =  6380.0 / 60 * SdsModuleConfigurations.MK4_L1.getDriveReduction() * SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

  public static final double Max_Angle = Max_Velocity / Math.hypot(Constants.trackwidth, Constants.wheelBase);




  public final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
    //divide by 2 if doesn't work
    new Translation2d(Constants.trackwidth, Constants.wheelBase),
    //^Front left wheel
    // Front Right
    new Translation2d(Constants.trackwidth, -Constants.wheelBase),
    //Back left
    new Translation2d(-Constants.trackwidth, Constants.wheelBase),
    //back Right
    new Translation2d(-Constants.trackwidth, -Constants.wheelBase)  
  );

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

 // int FrontLdrive = Front_leftDRIVEMotor;
  

  private ChassisSpeeds baseSpeed = new ChassisSpeeds(0,0,0);
 
  public SwerveSubsystem() {
    frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(Mk4SwerveModuleHelper.GearRatio.L1,Constants.Front_LeftDRIVEMotor,Constants.Front_LeftSTEERMotor,Constants.Front_LeftSTEER_Encoder,Constants.Front_LeftSTEER_Offset);
        //sees current state of module
    frontRightModule = Mk4SwerveModuleHelper.createFalcon500(Mk4SwerveModuleHelper.GearRatio.L1, Constants.Front_RightDRIVEMotor,Constants.Front_RightSTEERMotor,Constants.Front_RightSTEER_Encoder,Constants.Front_RightSTEER_Offset);
    backLeftModule = Mk4SwerveModuleHelper.createFalcon500(Mk4SwerveModuleHelper.GearRatio.L1, Constants.Back_LeftDRIVEMotor, Constants.Back_LeftSTEERMotor, Constants.Back_LeftSTEER_Encoder, Constants.Back_LeftSTEER_Offset);
    backRightModule = Mk4SwerveModuleHelper.createFalcon500(Mk4SwerveModuleHelper.GearRatio.L1, Constants.Back_RightDRIVEMotor, Constants.Back_RightSTEERMotor, Constants.Back_RightSTEER_Encoder, Constants.Back_RIGHTSTEER_Offset);


}

  public void drive(ChassisSpeeds chassisSpeeds){
    baseSpeed = chassisSpeeds;
  }
  

  public void zeroGyro(boolean boom){
    pigeon.setFusedHeading(0.0);
  }
  public Rotation2d getGyro(){
    return Rotation2d.fromDegrees(pigeon.getFusedHeading());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SwerveModuleState[] states = SwerveKinematics.toSwerveModuleStates(baseSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Max_Velocity);

    frontLeftModule.set(states[0].speedMetersPerSecond / Max_Velocity * Voltage, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / Max_Velocity * Voltage, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / Max_Velocity * Voltage, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / Max_Velocity * Voltage, states[3].angle.getRadians());

    SmartDashboard.putNumber("One Encoder", test1.getAbsolutePosition());
    SmartDashboard.putNumber("Two Encoder", test2.getAbsolutePosition());
    SmartDashboard.putNumber("Four Encoder", test3.getAbsolutePosition());
    SmartDashboard.putNumber("Three Encoder", test4.getAbsolutePosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
