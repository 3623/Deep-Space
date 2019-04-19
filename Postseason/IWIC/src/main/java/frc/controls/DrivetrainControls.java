package frc.controls;

import java.io.IOException;

import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Tuple;
import frc.util.Utils;

public class DrivetrainControls {
	public static double turnToAngle(double goal, double heading) {
		double error;
		double ROTATION_kP = 4.0;
		
		double difference = heading-goal;
		if(difference > 180){
			// When the values cross to and from 0 & 360, the gross difference is greater than 180
			error = difference-360;
		} else if(difference < -180){
			error = difference+360;
		} else{
			error = difference;
		}

		// Sets output rotation to inverted dif as a factor of the given magnitude
		// Uses cbrt to give greater output at mid to low differences
		double output = 0.4*Math.cbrt(-1*error/180*ROTATION_kP);
		// Deadband-ish
		if(Utils.withinThreshold(error, 0.0, 4.0)) output = error/-180*ROTATION_kP;
		return output;
	}

	public static Tuple arcadeDrive(double xSpeed, double zRotation) {	
		xSpeed = Math.max(-1.0, Math.min(1.0, xSpeed));
	
		zRotation = Math.max(-1.0, Math.min(1.0, zRotation));
	
		double leftMotorOutput;
		double rightMotorOutput;
	
		double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
	
		if (xSpeed >= 0.0) {
		  // First quadrant, else second quadrant
		  if (zRotation >= 0.0) {
			leftMotorOutput = maxInput;
			rightMotorOutput = xSpeed - zRotation;
		  } else {
			leftMotorOutput = xSpeed + zRotation;
			rightMotorOutput = maxInput;
		  }
		} else {
		  // Third quadrant, else fourth quadrant
		  if (zRotation >= 0.0) {
			leftMotorOutput = xSpeed + zRotation;
			rightMotorOutput = maxInput;
		  } else {
			leftMotorOutput = maxInput;
			rightMotorOutput = xSpeed - zRotation;
		  }
		}
	
		return new Tuple(leftMotorOutput, rightMotorOutput);
	  }

	public static void main ( String[] args ) throws IOException {
	}
}
