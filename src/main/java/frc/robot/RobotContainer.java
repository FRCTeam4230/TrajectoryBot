// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPIDAgainstGravity;
import frc.robot.commands.ArmPIDWithGravity;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer {
  private DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);

  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final ArmPIDAgainstGravity scoreTop = new ArmPIDAgainstGravity(armSubsystem,
  () -> Constants.ArmPositions.SCORE_TOP);

  private final ArmPIDWithGravity bringIn = new ArmPIDWithGravity(armSubsystem, 
  () -> Constants.ArmPositions.BRING_IN);

  private final ArmPIDWithGravity pickUpFromGround = new ArmPIDWithGravity(armSubsystem, 
  () -> Constants.ArmPositions.PICK_UP_FROM_GROUND);

  

  public RobotContainer() {
    //Should show the robot moving on smart dashboard

    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
    new JoystickButton(controller, XboxController.Button.kA.value).onTrue(scoreTop);
    new JoystickButton(controller, XboxController.Button.kB.value).onTrue(bringIn);
    new JoystickButton(controller, XboxController.Button.kX.value).onTrue(pickUpFromGround);

  }

  private void configureDefaultCommands() {
    driveTrain.setDefaultCommand(new DriveCommand(driveTrain, () -> controller.getRightX(), () -> controller.getLeftY()));
  }

  
  public Command getAutonomousCommand() {
    Constants.TrajectoryConstants.AUTO_EVENT_MAP.put("raiseArm", scoreTop);

    var traj =
            PathPlanner.loadPath("testPath", new PathConstraints(0.7, 1));

    Command followPath = new PPRamseteCommand(
      traj,
      driveTrain::getPose,
      new RamseteController(),
      driveTrain.getFeedforward(), 
      driveTrain.getKinematics(), 
      driveTrain::getSpeeds, 
      driveTrain.getLeftPIDController(),
      driveTrain.getRightPIDController(),
      driveTrain::setOutput,
      driveTrain).beforeStarting(new InstantCommand(() -> driveTrain.resetOdometry(traj.getInitialPose())));

      Command command = new FollowPathWithEvents(followPath, traj.getMarkers(), Constants.TrajectoryConstants.AUTO_EVENT_MAP);
      // new FollowPathWithEvents(command, null, null);

      return followPath;
  }
}
