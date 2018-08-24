package org.usfirst.frc.team3623.simulation;

import org.usfirst.frc.team3623.robot.util.CartesianCoordinate;
import org.usfirst.frc.team3623.simulation.motors.CIMMotor;

/**
 * 
 * @author eric
 *
 */
public class DrivetrainSide{
	CartesianCoordinate position;
	double velocity;
	double acceleration;
	double psuedoMass;
	
	private static final double WHEEL_RADIUS = 0.0508; // meters
	private static final double CIMS_PER_SIDE = 2.0; // Minicim is 0.58
	private static final double GEAR_RATIO = 10.7/1.0; // Reduction
	// Future can make motors a list of motors rather than coefficient
//	private static final CIMMotor cimL1, cimL2, cimR1, cimR2 = new CIMMotor();
//	private static final Motor[] motorsL = {cimL1, cimL2};
	
	static CIMMotor cim = new CIMMotor();
	
	public DrivetrainSide(CartesianCoordinate position, double mass) {
		this.position = position;
		velocity = 0.0;
		acceleration = 0.0;
		psuedoMass = mass;
	}
	
	public void update(double voltage, double time) {
		double motorSpeed = this.wheelSpeedToMotorSpeed(this.velocity);
		double newAcceleration = this.wheelAcceleration(voltage, motorSpeed);
		this.velocity += (newAcceleration + this.acceleration) / 2 * time; // Trapezoidal integration
		this.acceleration = newAcceleration;

	}
	
	/**
	 * 
	 * @param voltage input voltage to each motor
	 * @param speed initial motor speed, in RPM
	 * @return
	 */
	public double wheelAcceleration(double voltage, double speed) {
		double totalTorque = CIMMotor.outputTorque(voltage, speed) * GEAR_RATIO * CIMS_PER_SIDE;
		double wheelForce = (totalTorque / WHEEL_RADIUS);
		double wheelAcceleration = wheelForce / psuedoMass;
		return wheelAcceleration;
	}
	
	/**
	 * Converts linear wheel speed back to motor angular speed
	 * @param speed meters/sec
	 * @return angular speed, revolutions per minute
	 */
	private double wheelSpeedToMotorSpeed(double speed) {
		double wheelCircum = WHEEL_RADIUS * 2 * Math.PI;
		double wheelRevs = speed / wheelCircum * 60.0;
		double motorRevs = wheelRevs * GEAR_RATIO;
		return motorRevs;
	}
}