// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrainSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
  private DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  

  public RobotContainer() {
    //Should show the robot moving on smart dashboard
    Field2d field = new Field2d();
    SmartDashboard.putData(field);
    
    
    configureBindings();
  }

  private void configureBindings() {}

  
  public Command getAutonomousCommand() {

    PPRamseteCommand command = new PPRamseteCommand(
      PathPlanner.loadPath("testPath", new PathConstraints(1, 0.5)), 
      driveTrain::getPose, 
      new RamseteController(),
      driveTrain.getFeedforward(), 
      driveTrain.getKinematics(), 
      driveTrain::getSpeeds, 
      driveTrain.getLeftPIDController(),
      driveTrain.getRightPIDController(),
      driveTrain::setOutput,
      driveTrain);
    
      return command;
  }
}
