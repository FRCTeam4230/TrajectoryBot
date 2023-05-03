// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainSubsystemConstants;

public class DriveTrainSubsystem extends SubsystemBase {
  private CANSparkMax leftMaster = new CANSparkMax(DriveTrainSubsystemConstants.LEFT_MASTER_PORT, MotorType.kBrushless);
  private CANSparkMax rightMaster = new CANSparkMax(DriveTrainSubsystemConstants.RIGHT_MASTER_PORT, MotorType.kBrushless);

  private CANSparkMax leftSlave = new CANSparkMax(DriveTrainSubsystemConstants.LEFT_SLAVE_PORT, MotorType.kBrushless);
  private CANSparkMax rightSlave = new CANSparkMax(DriveTrainSubsystemConstants.RIGHT_SLAVE_PORT, MotorType.kBrushless);

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  //Takes in the robot's width, more specifically the distance between the wheels
  //Used to convert linear and angular velocity into left and right wheel speeds
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));

  //In the video this took in kinematics and getHeading()
  //Odometry calculates the robot's current position in the field
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), getLeftEncoder(), getRightEncoder());

  //Variable to store the position of the robot
  private Pose2d pose;

  //Feedforward uses constants to calculate how to power to give to the motors to move them a certain distance
  //Predicts how far robot will move, as opposed to pid, which reacts after the robot has moved
  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.DriveTrainSubsystemConstants.kS, Constants.DriveTrainSubsystemConstants.kV, Constants.DriveTrainSubsystemConstants.kV);

  //P is the only term needed because the setpoint is a velocity
  //Feedforward as the base speed, the pid controller to fine tune
  private PIDController leftPIDController = new PIDController(Constants.DriveTrainSubsystemConstants.kP, 0, 0);
  private PIDController rightPIDController = new PIDController(Constants.DriveTrainSubsystemConstants.kP, 0, 0);

  public DriveTrainSubsystem() {
    configMotors(leftMaster);
    configMotors(rightMaster);
    configMotors(leftSlave);
    configMotors(rightSlave);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
  }

  public Rotation2d getHeading() {
    //Negative because built in classes want it the other way
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public double getLeftEncoder() {
    return leftMaster.getEncoder().getPosition();
  }

  public double getRightEncoder() {
    return rightMaster.getEncoder().getPosition();
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      //getVelocity returns rotations per minute (rpm) of the motor
      //Gear ratio converts that into rpm of the wheel
      //2pi * radius converts rpm of the wheel to meters per minute  for velocity
      //Divide by 60 to get meters per second for velocity
      // leftMaster.getEncoder().getVelocity() / Constants.DriveTrainSubsystemConstants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(Constants.DriveTrainSubsystemConstants.WHEEL_RADIUS) / 60,
      // rightMaster.getEncoder().getVelocity() / Constants.DriveTrainSubsystemConstants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(Constants.DriveTrainSubsystemConstants.WHEEL_RADIUS) / 60
      leftMaster.getEncoder().getVelocity() * Units.inchesToMeters(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES) / 60,
      rightMaster.getEncoder().getVelocity() * Units.inchesToMeters(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES) / 60
    );
  }


  public SimpleMotorFeedforward getFeedforward() {
    return feedForward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setOutput(double leftVolts, double rightVolts) {
    //Volts to from -12 to 12
    //set takes in -1 to 1
    //Divide by 12 converts the number from -12 to 12 range to -1 to 1 range
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }

  @Override
  public void periodic() {
    //Updates robot position, in the video it took getHeading() and getSpeeds()
    //Here it takes in the distance travelled instead of velocity, which is kinda weirds
     pose = odometry.update(getHeading(),
     Units.inchesToMeters(getRightEncoder()),
     Units.inchesToMeters(getLeftEncoder()));
  }

  private void configMotors(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(Constants.DriveTrainSubsystemConstants.RAMP_RATE);
    motor.setIdleMode(IdleMode.kCoast);
    motor.getEncoder().setPositionConversionFactor(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES);
  }
}
