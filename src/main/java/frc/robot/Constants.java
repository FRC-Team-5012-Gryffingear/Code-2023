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

    public static final double trackwidth = 0.5969;
    public static final double wheelBase = 0.5969;
  
    public static final int driverController = 0;
    public static final int operatorController = 1;

//fix id
    public static final int pigeonID = 0;

    public static final int Front_LeftDRIVEMotor = 10;
    public static final int Front_LeftSTEERMotor = 0;
    public static final int Front_LeftSTEER_Encoder = 0;
    public static final double Front_LeftSTEER_Offset = -Math.toRadians(45.0);

    public static final int Front_RightDRIVEMotor =10;
    public static final int Front_RightSTEERMotor = 10;
    public static final int Front_RightSTEER_Encoder = 10;
    public static final double Front_RightSTEER_Offset = -Math.toRadians(45.0);

    public static final int Back_LeftDRIVEMotor = 0;
    public static final int Back_LeftSTEERMotor = 0;
    public static final int Back_LeftSTEER_Encoder = 0;
    public static final double Back_LeftSTEER_Offset = -Math.toRadians(45.0);
    
    public static final int Back_RightDRIVEMotor = 0;
    public static final int Back_RightSTEERMotor = 0;
    public static final int Back_RightSTEER_Encoder = 0;
    public static final double Back_RIGHTSTEER_Offset = -Math.toRadians(45.0);
  
}
