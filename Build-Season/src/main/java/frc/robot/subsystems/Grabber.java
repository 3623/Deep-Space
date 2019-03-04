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


public class Grabber {
    Solenoid clawSolenoid1 = new Solenoid(1);
    Solenoid clawSolenoid2 = new Solenoid(2);

    Solenoid extensionSolenoid = new Solenoid(3);
    
    DigitalInput hatchSwitch = new DigitalInput(8);

    Boolean isPlacing = false;

    public void intake(){
        extensionSolenoid.set(true);
        if (hatchSwitch.get()) openClaw();
        else closeClaw();
        isPlacing = false;
    }

    public void place(){
        extensionSolenoid.set(true);
        halfClaw();
        isPlacing = true;
    }

    public void defaultState(){
        extensionSolenoid.set(false);
        if (hatchSwitch.get()){
            if (isPlacing) halfClaw();
            else openClaw();
        } else {
            closeClaw();
            isPlacing = false;
        }
    }

    
    private void openClaw() {
        clawSolenoid1.set(true);
        clawSolenoid2.set(true);
    }

    private void closeClaw() {
        clawSolenoid1.set(false);
        clawSolenoid2.set(false);
    }

    private void halfClaw() {
        clawSolenoid1.set(true);
        clawSolenoid2.set(false);
    }
}
