package org.usfirst.frc.team3623.robot.subsystems;

import org.usfirst.frc.team3623.robot.util.CartesianCoordinate;
import org.usfirst.frc.team3623.robot.util.Geometry;
import org.usfirst.frc.team3623.robot.util.Tuple;

public class Drivetrain {
	private static final double WHEEL_BASE = 0.762; // meters

	
	
	public static Tuple driveToPoint(CartesianCoordinate goal, CartesianCoordinate current) {
		CartesianCoordinate leftGoal = Geometry.inverseCenterLeft(goal, WHEEL_BASE);
		CartesianCoordinate rightGoal = Geometry.inverseCenterRight(goal, WHEEL_BASE);
		CartesianCoordinate left = Geometry.inverseCenterLeft(current, WHEEL_BASE);
		CartesianCoordinate right = Geometry.inverseCenterRight(current, WHEEL_BASE);
		double distLeft = Geometry.distance(leftGoal, left);
		double distRight = Geometry.distance(rightGoal, right);
//		System.out.println(distLeft + " " + distRight);
		Tuple out = new Tuple(distLeft, distRight);
		return out;
	}
	
	public static Tuple driveToPoint2(CartesianCoordinate goal, CartesianCoordinate current) {
		CartesianCoordinate left = Geometry.inverseCenterLeft(current, WHEEL_BASE);
		CartesianCoordinate right = Geometry.inverseCenterRight(current, WHEEL_BASE);
		double distLeft = Geometry.distance(goal, left);
		double distRight = Geometry.distance(goal, right);
		System.out.println(distLeft + " " + distRight);
		Tuple out = new Tuple(distLeft, distRight);
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
