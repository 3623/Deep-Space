/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Grabber extends Subsystem {
    Solenoid Solenoid1 = new Solenoid(1);
    Solenoid Solenoid2 = new Solenoid(2);

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
    }
    
    public void openClaw() {
        Solenoid1.set(true);
        Solenoid2.set(true);
    }

    public void closeClaw() {
        Solenoid1.set(false);
        Solenoid2.set(false);
    }

    public void halfClaw() {
        Solenoid1.set(true);
        Solenoid2.set(false);
    }
}
