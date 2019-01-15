package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain {
	Spark frontRight = new Spark(0);
	Spark backRight = new Spark(1);
	Spark frontLeft = new Spark(2);
	Spark backLeft = new Spark(3);
	
	SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeft, backLeft);
	SpeedControllerGroup  rightMotors = new SpeedControllerGroup(frontRight, backRight);
	
	Encoder encLeft = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder encRight = new Encoder(1, 2, false, Encoder.EncodingType.k4X);
	
	DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);
	
	public void writeToLog() {
    };

//    public void outputToSmartDashboard {	}

    public void stop() {
    	leftMotors.disable();
    	rightMotors.disable();
	}

//    public void zeroSensors() {
//    	encLeft.reset();
//    	encRight.reset();	
//	}
    
   public void openLoopControl(double xSpeed, double rSpeed) {
	   drivetrain.curvatureDrive(xSpeed, rSpeed, false);
   }

//    public void registerEnabledLoops(Looper enabledLooper) {
//	}
 
    
    public static void main(String[] args) throws IOException{}
    







}