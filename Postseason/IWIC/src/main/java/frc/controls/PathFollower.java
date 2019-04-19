package frc.controls;

import java.io.IOException;

import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Tuple;
import frc.util.Utils;

public class PathFollower {
	private static final double WHEEL_BASE = 0.762; // meters

	
	
	public static Tuple driveToPoint(Pose goal, Pose current) {
		Pose leftGoal = Geometry.inverseCenterLeft(goal, WHEEL_BASE);
		Pose rightGoal = Geometry.inverseCenterRight(goal, WHEEL_BASE);
		Pose left = Geometry.inverseCenterLeft(current, WHEEL_BASE);
		Pose right = Geometry.inverseCenterRight(current, WHEEL_BASE);
		double distLeft = Geometry.distance(leftGoal, left);
		double distRight = Geometry.distance(rightGoal, right);
//		System.out.println(distLeft + " " + distRight);
		Tuple out = new Tuple(distLeft, distRight);
		return out;
	}
	
	public static Tuple driveToPoint2(Waypoint goal, Pose current) {
		Pose goalCenter = new Pose(goal.x, goal.y, goal.heading);
		Pose leftGoal = Geometry.inverseCenterLeft(goalCenter, WHEEL_BASE);
		Pose rightGoal = Geometry.inverseCenterRight(goalCenter, WHEEL_BASE);
		Pose left = Geometry.inverseCenterLeft(current, WHEEL_BASE);
		Pose right = Geometry.inverseCenterRight(current, WHEEL_BASE);
		double distLeft = Geometry.distance(leftGoal, left);
		double distRight = Geometry.distance(rightGoal, right);
		
		double direction;
		if (goal.driveBackwards) direction = -1.0;
		else direction = 1.0;
		double leftOut = distLeft*direction*goal.kSpeedFactor;
		double rightOut = distRight*direction*goal.kSpeedFactor;
		// System.out.println(leftOut + " " + rightOut);

		Tuple out = new Tuple(leftOut, rightOut);
		return out;
	}
	
	public static Tuple deadReckoningCurveLeft(double time) {
		double leftVoltage;
		double rightVoltage;
		if (time < 0.3) {
			rightVoltage = 0.0;
			leftVoltage = 0.0;
		} else if (time < 0.6) {
			rightVoltage = 12.0;
			leftVoltage = 5.0;
		} else if (time < 1.2) {
			rightVoltage = 6.0;
			leftVoltage = 6.0;
		} else if (time < 1.9) {
			rightVoltage = 3.35;
			leftVoltage = 9.65;
		} else {
			rightVoltage = 0.0;
			leftVoltage = 0.0;
		}
		Tuple out = new Tuple (leftVoltage, rightVoltage);
		return out;
	}
	
	public static Tuple deadReckoningStraight(double time) {
		double leftVoltage;
		double rightVoltage;
		if (time < 2.0) {
		rightVoltage = 12.0;
		leftVoltage = 12.0;
	} else {
		rightVoltage = 0.0;
		leftVoltage = 0.0;
	}
		Tuple out = new Tuple (leftVoltage, rightVoltage);
		return out;
	}

	public static double sCurve(Pose currentPose, Pose startPoint, Pose goalPoint){
		double distS = Math.sqrt(Math.pow(startPoint.x - currentPose.x, 2) + Math.pow(startPoint.y - currentPose.y, 2));
		double angleS = Math.atan2(startPoint.x-currentPose.x, startPoint.y-currentPose.y);
		double relativeAngleS = angleS - startPoint.r;
		double lineDistS = distS*Math.sin(relativeAngleS);
		double collinearDistS = distS*Math.cos(relativeAngleS);
		double xChaseS = startPoint.x - (collinearDistS * Math.sin(startPoint.r));
		double yChaseS = startPoint.y - (collinearDistS * Math.cos(startPoint.r));
		double chaseRelativeAngleS = Math.atan2(xChaseS-currentPose.x, yChaseS-currentPose.y);
		double chaseAngleS = startPoint.r + (chaseRelativeAngleS)- currentPose.r;
		System.out.println(chaseRelativeAngleS);
		double chaseAngleDegS = Math.toDegrees(chaseAngleS);

		double distG = Math.sqrt(Math.pow(goalPoint.x - currentPose.x, 2) + Math.pow(goalPoint.y - currentPose.y, 2));
		double angleG = Math.atan2(goalPoint.x-currentPose.x, goalPoint.y-currentPose.y);
		double relativeAngleG = angleG - goalPoint.r;
		double lineDistG = distG*Math.sin(relativeAngleG);
		double collinearDistG = distS*Math.cos(relativeAngleS);
		double xChaseG = goalPoint.x - (collinearDistG * Math.sin(goalPoint.r));
		double yChaseG = goalPoint.y - (collinearDistG * Math.cos(goalPoint.r));
		double chaseRelativeAngleG = Math.atan2(xChaseG-currentPose.x, yChaseG-currentPose.y);
		double chaseAngleG = goalPoint.r + (chaseRelativeAngleG) - currentPose.r;
		double chaseAngleDegG = Math.toDegrees(chaseAngleG);

		double weightS = lineDistG/(lineDistS+lineDistG);
		double weightG = lineDistS+0.0/(lineDistS+lineDistG);

		double output = (chaseAngleDegS*weightS + chaseAngleDegG*weightG);
		System.out.println("Distance S: " + lineDistS + ", Distance G: " + lineDistG);
		System.out.println("Weight S: " + weightS + ", Weight G: " + weightG);
		System.out.println("Chase Angle S: " + chaseAngleDegS + ", Chase Angle G: " + chaseAngleDegG + ", Weight Chase Angle: " + output);

		return output;
	} 

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
	  Pose pose = new Pose(1.0, 0.0, 45.0);
	  Pose startPoint = new Pose(0.0, 0.0, 45.0);
	  Pose goalPoint = new Pose(1.0, 1.0, 0.0);
	  double goalAngle = PathFollower.sCurve(pose, startPoint, goalPoint);
	}
}
