package org.usfirst.frc.team3623.simulation;

import org.usfirst.frc.team3623.simulation.motors.CIMMotor;
import org.usfirst.frc.team3623.simulation.motors.Motor;
import org.usfirst.frc.team3623.simulation.CartesianCoordinate;
import org.usfirst.frc.team3623.simulation.Kinematics;
import org.usfirst.frc.team3623.simulation.DrivetrainSide;

/**
 * Model for a differential drivetrain, simulates mass and motor performance
 * Integrates acceleration over time for both sides of the drivetrain and uses velocity to 
 * calculate the instantaneous curvature of the robot, which is then integrated for position
 * @author eric
 *
 */
public class DrivetrainModel {

	private static final double DRIVETRAIN_MASS = 63.5; // kg
	private static final double WHEEL_BASE = 0.762; // meters
	private static final double CENTER_MASS = 0.381; // from left wheel
	public CartesianCoordinate center;
	private DrivetrainSide left, right;
	
	public DrivetrainModel(double x, double y, double heading) {
		center = new CartesianCoordinate(x, y, heading); // Initial robot position
		left = new DrivetrainSide(Geometry.inverseCenterLeft(center, WHEEL_BASE),
				DRIVETRAIN_MASS/2);
		right = new DrivetrainSide(Geometry.inverseCenterRight(center, WHEEL_BASE),
				DRIVETRAIN_MASS/2);	
	}
	
	public DrivetrainModel() {
		center = new CartesianCoordinate(0.0, 0.0, 0.0); // Initial robot position
		left = new DrivetrainSide(Geometry.inverseCenterLeft(center, WHEEL_BASE),
				DRIVETRAIN_MASS/2);
		right = new DrivetrainSide(Geometry.inverseCenterRight(center, WHEEL_BASE),
				DRIVETRAIN_MASS/2);	
	}

	
	public void update(double lVoltage, double rVoltage, double time) {
		left.update(lVoltage, time);
		right.update(rVoltage, time);

		double radius = Kinematics.radiusICC(WHEEL_BASE, left.velocity, right.velocity);
		double omega = Kinematics.velocityICC(WHEEL_BASE, left.velocity, right.velocity);
		double theta = omega * (time);
		double sinTheta = Math.sin(theta);
		double alpha = (Math.PI/4.0) - (theta/2.0);
		double sinAlpha = Math.sin(alpha);

		double movementAngle = center.heading - (theta/2.0);
		double leftMovement = Geometry.sideFromLawOfSines(radius+(WHEEL_BASE/2), sinAlpha, sinTheta);
		double rightMovement = Geometry.sideFromLawOfSines(radius-(WHEEL_BASE/2), sinAlpha, sinTheta);
		
		if (omega == 0.0) {
			leftMovement = -left.velocity * time;
			rightMovement = -right.velocity * time;
		}
		double sine = Math.sin(movementAngle);
		double cosine = Math.cos(movementAngle);
		double leftMovementX = -leftMovement*sine;
		double leftMovementY = -leftMovement*cosine;
		left.position.update(leftMovementX, leftMovementY);
		double rightMovementX = -rightMovement*sine;
		double rightMovementY = -rightMovement*cosine;
		right.position.update(rightMovementX, rightMovementY);
		center = Geometry.center(left.position, right.position);
		
		// Debug statements
//		System.out.println(center.x + ", " + center.y + ", " + center.heading);
		System.out.println("LV: " + left.velocity + "RV: " + right.velocity);
//		System.out.println("LP: " + left.position.x + ", " + left.position.y);
//		System.out.println("RP: " + right.position.x + ", " + right.position.y);
//		System.out.println("MA: " + movementAngle + "Rad ICC: " + radius);
//		System.out.println("RM: " + rightMovement + " LM: " + leftMovement);
//		System.out.println(leftMovementX + "====" + leftMovementY);
//		System.out.println(rightMovementX + "====" + rightMovementY);
	}
	
	// Testing
	public static void main ( String[] args) {
		DrivetrainModel model = new DrivetrainModel(0.0, 0.0, 0.0);
		for (int i = 0; i < 300; i++) {
			model.update(12.0, 12.0, 10.0/1000.0);
		}
	}
}
