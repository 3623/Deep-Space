package org.usfirst.frc.team3623.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class DriveTrain {
	Spark frontLeft = new Spark(0);
	Spark frontRight = new Spark(1);
	Spark backLeft = new Spark(2);
	Spark backRight = new Spark(3);
	
	SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeft, backLeft);
	SpeedControllerGroup  rightMotors = new SpeedControllerGroup(frontRight, backRight);
	
	
	public void writeToLog() {
    };

    public void outputToSmartDashboard {	}

    public int stop() {
    	stopMotor();
		return 0;
	}

    public void zeroSensors() {
	}

//    public void registerEnabledLoops(Looper enabledLooper) {
//	}
 
 
    
    public static void main(String[] args) throws IOException{}
    







}