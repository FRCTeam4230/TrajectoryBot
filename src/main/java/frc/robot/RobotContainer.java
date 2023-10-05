// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
  

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {}

  private void configureDefaultCommands() {
    driveTrain.setDefaultCommand(new DriveCommand(driveTrain, () -> controller.getRightX(), () -> controller.getLeftY()));
  }

  
  public Command getAutonomousCommand() {

    var traj =
            PathPlanner.loadPath("testPath", new PathConstraints(0.5, 0.5));

    Command command = new PPRamseteCommand(
      traj,
      driveTrain::getPose,
      new RamseteController(),
      driveTrain.getFeedforward(), 
      driveTrain.getKinematics(), 
      driveTrain::getSpeeds, 
      driveTrain.getLeftPIDController(),
      driveTrain.getRightPIDController(),
      driveTrain::setOutput,
      driveTrain)
            .beforeStarting(new InstantCommand(() -> driveTrain.resetOdometry(traj.getInitialPose())));

      return command;
  }
}
