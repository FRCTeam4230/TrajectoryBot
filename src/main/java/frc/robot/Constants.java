// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;

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
    public static final double kS = 0.12941;//Bubblegum

//    public static final double kV = 0.15344;//2022's bot
    public static final double kV = 2.7353;//Bubblegum

//    public static final double kA = 0.032907;//2022's bot
    public static final double kA = 0.41063;//Bubblegum

      //    public static final double kP = 2;//2022's bot
    // public static final double kP = 2.4077;//Bubblegum
    public static final double kP = 0.0027015;//Bubblegum

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

    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

  }

  public static class Arm {
    public static final int ARM_MOTOR_ID = 4;
    public static final double ARM_RAMP_RATE = 0.5;
    // Might change encoder to more accurate. Update the number 42 is we get new
    // encoder
    // public static final double MOTOR_TO_DEGREES = ((1 / 42.0) * (1 / 48.0) * 360);
    // public static final double MOTOR_TO_DEGREES = 15.498288387457226 / 1.045917510986328;
    public static final double MOTOR_TO_DEGREES = 1;

    public static final int ENCODER_PORT = 0;
    public static final int FRONT_LIMIT_PORT = 1;
    public static final int BACK_LIMIT_PORT = 2;
    public static final double FORWARD_LIMIT_ANGLE = 279;
    public static final double BACK_LIMIT_ANGLE = 0.5;
    public static final double ARM_SPEED = 0.2;

    //Constants for arm zones
    public static final double BOUNDARY_FAST_MINIMUM = 5;
    public static final double BOUNDARY_FAST_MAXIMUM = 255;
    public static final double ENTERING_ROTATION_SAFETY_ZONE_LIMIT = 0.8;
    public static final double EXITING_ROTATION_SAFETY_ZONE_LIMIT = 0.85;


}

public static class ArmPIDConstants {

  public static final double kP_WITH_GRAVITY = 0.005;
  public static final double kI_WITH_GRAVITY = 0;
  public static final double kD_WITH_GRAVITY = 0;

  public static final double kP_AGAINST_GRAVITY = 0.03;
  public static final double kI_AGAINST_GRAVITY = 0;
  public static final double kD_AGAINST_GRAVITY = 0;

  public static final double VELOCITY_TOLERANCE = 0;
  public static final double POSITION_TOLERANCE = 0.5;
  public static final double RANGE = 0.2;

}

public static class ArmPositions {

  // When the arm is inside the robot
  public static final double BRING_IN = 1.5;
  // When the robot scores in top row
  public static final double SCORE_TOP = 193;
  //When the robot scores in the middle row
  public static final double SOCRE_MIDDLE = 210;
  // When the robot is picking stuff off of the ground
  public static final double PICK_UP_FROM_GROUND = 284;
  // When the robot is picking stuff from the loading station
  public static final double PICK_UP_FROM_STATION = 195;

}

  public enum MotorID {
    ARM_MOTOR_ID(6, 0.5, IdleMode.kBrake, 1.0);

    private Integer id;
    private Double rampRate;
    private IdleMode idleMode;
    private Double positionConversionFactor;

    private MotorID(Integer id, Double rampRate, IdleMode idleMode, Double positionConversionFactor) {
        this.id = id;
        this.rampRate = rampRate;
        this.idleMode = idleMode;
        this.positionConversionFactor = positionConversionFactor;
    }
    
    public Integer getId() {
      return id;
  }

  public Double getRampRate() {
      return rampRate;
  }

  public Double getPositionConversionFactor() {
      return this.positionConversionFactor;
  }

  public IdleMode getIdleMode() {
      return idleMode;
  }
  }
}
