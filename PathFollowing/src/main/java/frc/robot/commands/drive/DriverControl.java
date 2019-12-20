/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriverControl extends Command {
  public DriverControl() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Boolean quickTurn;
    double driverYAxis = Robot.oi.driverController.getRawAxis(1);
    double driveTwistAxis = Robot.oi.driverController.getRawAxis(4);
    if (Math.abs(driverYAxis) < 0.5)
      quickTurn = true;
    else
      quickTurn = false;
    // quickTurn = true;

    if (quickTurn) {
      Robot.drivetrain.driverControl(-driverYAxis * 0.5, driveTwistAxis, quickTurn);
    } else {
      Robot.drivetrain.driverControl(-driverYAxis * Math.abs(driverYAxis), driveTwistAxis * 0.5, quickTurn);
    }

    if (Robot.oi.driverController.getXButton()) {
      Robot.drivetrain.zeroSensors();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
