/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import static frc.robot.Constants.*;
import frc.robot.RobotContainer;

public class DriveOffLinePath extends CommandBase {

  RamseteCommand command;
  /**
   * Creates a new AutonomousCommand.
   */
  Trajectory trajectory;
  
  public DriveOffLinePath() {
    final TrajectoryConfig config = new TrajectoryConfig(
        Units.feetToMeters(MAX_VELOCITY_FEET), Units.feetToMeters(MAX_ACCELERATION_FEET));
    config.setKinematics(RobotContainer.drive.getKinematics());

    final String trajectoryJSON = "PathWeaver/Paths/First.path";
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    command = new RamseteCommand(
      trajectory,
      RobotContainer.drive::getPose,
      new RamseteController(2, .7),
      RobotContainer.drive.getFeedforward(),
      RobotContainer.drive.getKinematics(),
      RobotContainer.drive::getSpeeds,
      RobotContainer.drive.getLeftPIDController(),
      RobotContainer.drive.getRightPIDController(),
      RobotContainer.drive::setOutputVolts,
      RobotContainer.drive
  );

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    RobotContainer.drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
