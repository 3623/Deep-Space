/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.controls.Waypoint;
import frc.robot.Robot;

public class LeftFarRocketToLeftLoadingZone extends Command {
  public LeftFarRocketToLeftLoadingZone() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.waypointNav.clearWaypoints();
    Robot.drivetrain.waypointNav.addWaypoint(new Waypoint(Robot.drivetrain.model.center));
    Robot.drivetrain.waypointNav.addWaypoint(new Waypoint(1.5, 5.5, 0.0, 0.7, 0.9, 0.9, true));
    Robot.drivetrain.waypointNav.addWaypoint(new Waypoint(0.6, 1.0, 0.0, 1.0, 1.3, 0.4, true));
    Robot.drivetrain.waypointNav.addWaypoint(new Waypoint(0.6, 0.4, 0.0, 0.2, 0.4, 0.3, true));
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drivetrain.driveToWaypoint();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drivetrain.waypointNav.getIsFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
