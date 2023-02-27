// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.swerveCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    //in meters
    public static final double trackwidth = 0.5207;
    public static final double wheelBase = 0.6541;
  // 0.65405 if needed to change wheelbase 
    public static final int driverController = 0;
    public static final int operatorController = 1;

// FIX ME IDS
    public static final int pigeonID = 0;

    public static final int Front_LeftDRIVEMotor = 1;
    public static final int Front_LeftSTEERMotor = 2;
    public static final int Front_LeftSTEER_Encoder = 3;
    public static final double Front_LeftSTEER_Offset = -Math.toRadians(322.2 - 90);

    public static final int Front_RightDRIVEMotor =4;
    public static final int Front_RightSTEERMotor = 5;
    public static  int Front_RightSTEER_Encoder = 6;
    public static double Front_RightSTEER_Offset = -Math.toRadians(173.4 + 90);

    public static final int Back_LeftDRIVEMotor = 7;
    public static final int Back_LeftSTEERMotor = 8;
    public static  int Back_LeftSTEER_Encoder = 9;
    public static double Back_LeftSTEER_Offset = -Math.toRadians(37.2 + 90);
    
    public static final int Back_RightDRIVEMotor = 10;
    public static final int Back_RightSTEERMotor = 11;
    public static  int Back_RightSTEER_Encoder = 12;
    public static final double Back_RIGHTSTEER_Offset = -Math.toRadians(245.4 + 90);
  
}     
