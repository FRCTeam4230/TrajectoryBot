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
  public static class DriveTrainSubsystemConstants {

    //This might be 3.33 : 1
    public static final double GEAR_RATIO = 0;
    public static final double WHEEL_RADIUS = 3.0;
    //Use "frc-characterization" tool to figure this out
    //That tool can also estimate the p term for a pid controller
//    public static final double kS = 0.29321; //2022's bot
    public static final double kS = 0.12731;//Bubblegum

//    public static final double kV = 0.15344;//2022's bot
    public static final double kV = 0.069002;//Bubblegum

//    public static final double kA = 0.032907;//2022's bot
    public static final double kA = 0.032907;//Bubblegum

      //    public static final double kP = 2;//2022's bot
    public static final double kP = 2.4077;//Bubblegum

//      public static final double kD = 0.39408;//2022's bot
      public static final double kD = 0.21483;//Bubblegum
    public static final double RAMP_RATE = 0.1;
    public static final double MOTOR_ROTATION_TO_INCHES = 72 / 40.687;

    public static final int LEFT_MASTER_PORT = 1;
    public static final int LEFT_SLAVE_PORT = 2;
    public static final int RIGHT_MASTER_PORT = 3;
    public static final int RIGHT_SLAVE_PORT = 4;

    public static final double SPEED_MULTIPLIER = 0.6;
    public static final double ROTATION_MULTIPLIER = 0.4;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TrajectoryConstants {

    public static final double MAX_VELOCITY = 4;
    public static final double MAX_ACCELERATION = 1;

  }
}
