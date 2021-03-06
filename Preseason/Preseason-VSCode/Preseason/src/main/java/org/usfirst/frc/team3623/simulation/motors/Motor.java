package org.usfirst.frc.team3623.simulation.motors;

/**
 * Abstract class for motor implementation, with methods for calculating torque
 * and in the future tracking thermal charecteristics.
 * To implement, update constants in declarator and run calculateSlopes()
 * 
 * @author eric
 *
 */
abstract public class Motor {
	protected static double STALL_TORQUE; // N. m
	protected static double FREE_SPEED; // RPM
	protected static double STALL_CURRENT; // Amps
	protected static double FREE_CURRENT; // Amps
	private static final double MAX_VOLTAGE = 12.0; // Volts
	private static double kSlopeTorque;
	private static double kSlopeCurrent;

	// Init value function
	protected void calculateSlopes () {
		kSlopeTorque = -STALL_TORQUE / FREE_SPEED;
		kSlopeCurrent = - STALL_CURRENT / FREE_CURRENT;
	}

	public static double outputTorque (double voltage, double speed) {
		double stallTorque = STALL_TORQUE * (voltage / MAX_VOLTAGE);
		double torque = (speed * kSlopeTorque) + stallTorque;
		return torque;
	}
	
	
}
