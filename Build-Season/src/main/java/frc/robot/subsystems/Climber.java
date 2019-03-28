/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  private Solenoid frontPistons, backPistons;

  public Climber(){
    frontPistons = new Solenoid(4);
    backPistons = new Solenoid(5);
  }

  public void setFront(Boolean extended){
    frontPistons.set(extended);
  }

  public void setBack(Boolean extended){
    backPistons.set(extended);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
