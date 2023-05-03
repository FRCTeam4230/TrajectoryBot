// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrainSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
  private DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private TrajectoryConfig config;
  private Trajectory trajectory;

  public RobotContainer() {
    //Should show the robot moving on smart dashboard
    Field2d field = new Field2d();
    SmartDashboard.putData(field);
    
    
    configureBindings();

    //PathWeaver should be able to do all the trajectory generation for us, replacing this code
    //Takes in max velocity m/s and max acceleration m/s^2
    config = new TrajectoryConfig(Units.inchesToMeters(Constants.TrajectoryConstants.MAX_VELOCITY), 
    Units.inchesToMeters(Constants.TrajectoryConstants.MAX_ACCELERATION));
    config.setKinematics(driveTrain.getKinematics());

    //Takes in a list of waypoints and the config variable
    trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
      config
    );
  }

  private void configureBindings() {}

  
  public Command getAutonomousCommand() {


    RamseteCommand command = new RamseteCommand(
      trajectory,
      // trajectoryFromPathWeaver,
      // trajectory,
      driveTrain::getPose,
      //It is suggested to use these constants, idk what they do
      //If needed, we can read the documentation on what they do and adjust them
      new RamseteController(2.0, 0.7), 
      driveTrain.getFeedforward(), 
      driveTrain.getKinematics(), 
      driveTrain::getSpeeds, 
      driveTrain.getLeftPIDController(),
      driveTrain.getRightPIDController(), 
      driveTrain::setOutput, 
      //Requires the drive train
      driveTrain);

      return command;
  }
}
