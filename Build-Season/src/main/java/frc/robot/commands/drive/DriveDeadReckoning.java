/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveDeadReckoning extends TimedCommand {
  /**
   * Add your docs here.
   */
  double xSpeed;
  double rSpeed;
  Boolean quickTurn;

  public DriveDeadReckoning(double timeout, double xSpeed, double rSpeed, Boolean quickTurn) {
    super(timeout);
    requires(Robot.drivetrain);
    this.xSpeed = xSpeed;
    this.rSpeed = rSpeed;
    this.quickTurn = quickTurn;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drivetrain.openLoopControl(xSpeed, rSpeed, quickTurn);
  }

  // Called once after timeout
  @Override
  protected void end() {
    Robot.drivetrain.openLoopControl(0.0, 0.0, false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
