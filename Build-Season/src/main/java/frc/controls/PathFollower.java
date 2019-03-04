package frc.controls;

import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Tuple;

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
}
