// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static int DriverController = 0;
  public static int OperatorController = 1;
//cant be 7,8,10,2,4,5,15,1
// 3 = 15, 9 = 16
  public static final int NeoIntake1 = 3;
  //Port 3s motor direction POSITIVE should move outward and NEGATIVE should move inward
  public static final int NeoIntake2 = 9;

  //Port 14s motor direction should have POSITIVE moving outward and NEGATIVE inward
  public static final int NeoExtender1 = 14;
  public static final int NeoExtender2 = 13;
// ElevatorM1 should move POSITIVE upward and NEGATIVE downward
  public static final int ElevatorMotor1 = 0;
  public static final int ElevatorMotor2 = 0;



}
