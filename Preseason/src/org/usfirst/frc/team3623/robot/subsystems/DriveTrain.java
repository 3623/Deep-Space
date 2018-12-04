package org.usfirst.frc.team3623.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class DriveTrain {
	Spark frontLeft = new Spark(0);
	Spark frontRight = new Spark(1);
	Spark backLeft = new Spark(2);
	Spark backRight = new Spark(3);
	
	SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeft, backLeft);
	SpeedControllerGroup  rightMotors = new SpeedControllerGroup(frontRight, backRight);
	
	Encoder encLeft = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder encRight = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	
	
	public void writeToLog() {
    };

    public void outputToSmartDashboard {	}

    public void stop() {
    	leftMotors.disable();
    	rightMotors.disable();
	}

    public void zeroSensors() {
    	encLeft.reset();
    	encRight.reset();
    	
	}
    
   public void openLoopControl

//    public void registerEnabledLoops(Looper enabledLooper) {
//	}
 
    
    public static void main(String[] args) throws IOException{}
    







}