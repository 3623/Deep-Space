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
import frc.robot.subsystems.Drivetrain;

public class LeftHab1ToLeftFarRocket extends Command {
  public LeftHab1ToLeftFarRocket() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // We will only call this drive command first, so we set position to a standard value, not
    // get postion and set
    Robot.drivetrain.model.setPosition(2.85, 1.7, 0.0);
		Robot.drivetrain.waypointNav.addWaypoint(new Waypoint(2.85, 1.7, 0.0));
		Robot.drivetrain.waypointNav.addWaypoint(new Waypoint(2.85, 3.5, 0.0, 0.3, 0.5, 0.5, false));
		Robot.drivetrain.waypointNav.addWaypoint(new Waypoint(1.2, 6.4, 0.0, 1.0, 1.2, 0.4, false));
		Robot.drivetrain.waypointNav.addWaypoint(new Waypoint(0.3, 6.8, 0.0, 0.2, 0.5, 0.5, false));
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
