/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.util.Utils;

public class TurretManualControl extends Command {
  public TurretManualControl() {
    requires(Robot.turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Utils.outsideDeadband(Robot.operatorController.getRawAxis(4), 0.0, 0.3) ||
    Utils.outsideDeadband(Robot.operatorController.getRawAxis(5), 0.0, 0.3)) {
        // Setpoint control
      double goalAngle = (Math.toDegrees(Math.atan2(Robot.operatorController.getRawAxis(4), -Robot.operatorController.getRawAxis(5)))+360.0)%360.0;
      double robotAngle = Robot.drivetrain.model.center.heading;
      Robot.turret.setSetpoint(((goalAngle - robotAngle)+360.0)%360.0);
    } else if (Utils.outsideDeadband(Robot.operatorController.getRawAxis(0), 0.0, 0.2)) {
      // Manual control with pot
      Robot.turret.setSpeed(Robot.operatorController.getRawAxis(0));
    } else {
     // Robot.turret.setVisionControlled();
      Robot.turret.setSpeed(0.0);
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
