
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.grabber.DefaultState;


public class Grabber extends Subsystem{
    Solenoid clawSolenoid1, clawSolenoid2, extensionSolenoid;
    
    DigitalInput hatchSwitch;
    
    Boolean isPlacing = false;

    public Grabber(){
        clawSolenoid1 = new Solenoid(1);
        clawSolenoid2 = new Solenoid(2);
        extensionSolenoid = new Solenoid(3);

        hatchSwitch = new DigitalInput(8);
    }
    
    public void openClaw() {
        clawSolenoid1.set(false);
        clawSolenoid2.set(true);
    }

    public void closeClaw() {
        clawSolenoid1.set(true);
        clawSolenoid2.set(false);
    }

    public void halfClaw() {
        clawSolenoid1.set(false);
        clawSolenoid2.set(false);
    }

    public void extend(){
        extensionSolenoid.set(true);
    }

    public void retract(){
        extensionSolenoid.set(false);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DefaultState());
    }
}
