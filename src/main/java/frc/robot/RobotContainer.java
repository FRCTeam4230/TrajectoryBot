// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
  

  public RobotContainer() {
    //Should show the robot moving on smart dashboard
    Field2d field = new Field2d();
    SmartDashboard.putData(field);
    
    
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {}

  private void configureDefaultCommands() {
    driveTrain.setDefaultCommand(new DriveCommand(driveTrain, () -> controller.getRightX(), () -> controller.getLeftY()));
  }

  
  public Command getAutonomousCommand() {

    PPRamseteCommand command = new PPRamseteCommand(
      PathPlanner.loadPath("path2", new PathConstraints(0.1, 0.5)),
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
