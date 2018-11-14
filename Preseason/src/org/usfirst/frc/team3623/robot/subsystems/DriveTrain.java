package org.usfirst.frc.team3623.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Spark;

public class DriveTrain {
	Spark frontLeft = new Spark(0);
	Spark frontRight = new Spark(1);
	Spark backLeft = new Spark(2);
	Spark backRight = new Spark(3);
	
	
	public void writeToLog() {
    };

    public void outputToSmartDashboard() {
	}

    public int stop() {
		return 0;
	}

    public void zeroSensors() {
	}

//    public void registerEnabledLoops(Looper enabledLooper) {
//	}
 
 
    
    public static void main(String[] args) throws IOException{}
    







}