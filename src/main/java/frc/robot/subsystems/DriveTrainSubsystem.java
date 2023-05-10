// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainSubsystemConstants;

public class DriveTrainSubsystem extends SubsystemBase {
  private CANSparkMax left1 = new CANSparkMax(DriveTrainSubsystemConstants.LEFT_MASTER_PORT, MotorType.kBrushless);
  private CANSparkMax right1 = new CANSparkMax(DriveTrainSubsystemConstants.RIGHT_MASTER_PORT, MotorType.kBrushless);

  private CANSparkMax left2 = new CANSparkMax(DriveTrainSubsystemConstants.LEFT_SLAVE_PORT, MotorType.kBrushless);
  private CANSparkMax right2 = new CANSparkMax(DriveTrainSubsystemConstants.RIGHT_SLAVE_PORT, MotorType.kBrushless);

  private MotorControllerGroup leftGroup = new MotorControllerGroup(left1, left2);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(right1, right2);

  private DifferentialDrive differentialDrive;

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
  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.DriveTrainSubsystemConstants.kS, Constants.DriveTrainSubsystemConstants.kV);

  //P is the only term needed because the setpoint is a velocity
  //Feedforward as the base speed, the pid controller to fine tune
  private PIDController leftPIDController = new PIDController(Constants.DriveTrainSubsystemConstants.kP, 0, 0);
  private PIDController rightPIDController = new PIDController(Constants.DriveTrainSubsystemConstants.kP, 0, 0);

  public DriveTrainSubsystem() {
    configMotors(left1);
    configMotors(right1);
    configMotors(left2);
    configMotors(right2);

    
    rightGroup.setInverted(true);

    differentialDrive= new DifferentialDrive(leftGroup, rightGroup);

    odometry.resetPosition(getHeading(), getLeftEncoderMeters(), getLeftEncoder(), new Pose2d());

    SmartDashboard.putData(this);
  }

  public void arcadeDrive(double forward, double rotation) {
    differentialDrive.arcadeDrive(
      MathUtil.clamp(forward * DriveTrainSubsystemConstants.SPEED_MULTIPLIER, -0.99, 0.99)
    , MathUtil.clamp(rotation * DriveTrainSubsystemConstants.ROTATION_MULTIPLIER, -0.99, 0.99));
  }

  public Rotation2d getHeading() {
    //Negative because built in classes want it the other way
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public double getLeftEncoder() {
    return -(left1.getEncoder().getPosition() + left2.getEncoder().getPosition()) / 2.0;
  }

  public double getRightEncoder() {
    return (right1.getEncoder().getPosition() + right2.getEncoder().getPosition()) / 2.0;
  }

  public double getLeftEncoderMeters() {
    return Units.inchesToMeters(getLeftEncoder());
  }

  public double getRightEncoderMeters() {
    return Units.inchesToMeters(getRightEncoder());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      //getVelocity returns rotations per minute (rpm) of the motor
      //Gear ratio converts that into rpm of the wheel
      //2pi * radius converts rpm of the wheel to meters per minute  for velocity
      //Divide by 60 to get meters per second for velocity
      // left1.getEncoder().getVelocity() / Constants.DriveTrainSubsystemConstants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(Constants.DriveTrainSubsystemConstants.WHEEL_RADIUS) / 60,
      // right1.getEncoder().getVelocity() / Constants.DriveTrainSubsystemConstants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(Constants.DriveTrainSubsystemConstants.WHEEL_RADIUS) / 60
      -left1.getEncoder().getVelocity() * Units.inchesToMeters(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES) / 60,
      right1.getEncoder().getVelocity() * Units.inchesToMeters(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES) / 60
    );
  }

  public double getLeftSpeed() {
    double left1Speed = (left1.getEncoder().getVelocity() * Units.inchesToMeters(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES) / 60);
    double left2Speed = (left2.getEncoder().getVelocity() * Units.inchesToMeters(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES) / 60);

    return -(left1Speed + left2Speed) / 2.0;
  }

  public double getRightSpeed() {
    double right1Speed = (right1.getEncoder().getVelocity() * Units.inchesToMeters(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES) / 60);
    double right2Speed = (right2.getEncoder().getVelocity() * Units.inchesToMeters(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES) / 60);

    return (right1Speed + right2Speed) / 2.0;
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
    leftGroup.set(MathUtil.clamp(leftVolts / 12, -0.5, 0.5));
    rightGroup.set(MathUtil.clamp(rightVolts / 12, -0.5, 0.5));
  }

  @Override
  public void periodic() {
    //Updates robot position, in the video it took getHeading() and getSpeeds()
    //Here it takes in the distance travelled instead of velocity, which is kinda weirds
     pose = odometry.update(getHeading(), getRightEncoderMeters(), getLeftEncoderMeters());
  }

  private void configMotors(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(Constants.DriveTrainSubsystemConstants.RAMP_RATE);
    motor.setIdleMode(IdleMode.kCoast);
    motor.getEncoder().setPositionConversionFactor(DriveTrainSubsystemConstants.MOTOR_ROTATION_TO_INCHES);
    motor.getEncoder().setPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("left encoder", () -> getLeftEncoder(), null);
    builder.addDoubleProperty("right encoder", () -> getRightEncoder(), null);
    builder.addDoubleProperty("left speed", () -> getLeftSpeed(), null);
    builder.addDoubleProperty("right speed", () -> getRightSpeed(), null);
  }
}
