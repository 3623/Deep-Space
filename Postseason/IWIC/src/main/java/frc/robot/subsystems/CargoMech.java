/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;

/**
 * Add your docs here.
 */
public class CargoMech extends Subsystem {
  private Spark tilt, wheels;

  public CargoMech(){
    tilt = new Spark(3);
    wheels = new Spark(4);
  }

public void tiltClaw(double speed){
    if (Math.abs(speed)>0.1){
        tilt.set(speed);
    }
    else{
        tilt.set(0.05);
    }
}

public void intake(double speed){
    wheels.set(speed);
}

  @Override
  public void initDefaultCommand() {
    
  }
}