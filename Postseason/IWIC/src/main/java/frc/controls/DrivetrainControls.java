package frc.controls;

import java.io.IOException;

import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Tuple;
import frc.util.Utils;

public class DrivetrainControls {
	static double error = 0.0;
	static double lastError = 0.0;
	static final double ROTATION_kP = 12.0;
	static final double ROTATION_kD = 20.0;

	public static double turnToAngle(double goal, double heading) {
		
		heading = ((heading%360.0)+360.0)%360.0;
		goal = ((goal%360.0)+360.0)%360.0;

		System.out.println(Math.round(heading) + " " + Math.round(goal));

		double difference = heading - goal;
		if (difference > 180) {
			// When the values cross to and from 0 & 360, the gross difference is greater
			// than 180
			error = difference - 360;
		} else if (difference < -180) {
			error = difference + 360;
		} else {
			error = difference;
		}

		// Sets output rotation to inverted dif as a factor of the given magnitude
		// Uses cbrt to give greater output at mid to low differences
		double output = 0.4 * Math.cbrt(-1 * error / 180 * ROTATION_kP);
		// Deadband-ish
		if (Utils.withinThreshold(error, 0.0, 4.0))
			output = error / -180 * ROTATION_kP;
		double pVal = (error / -180 * ROTATION_kP);
		double dVal = - ((heading - lastError) * ROTATION_kD / 180.0);
		output = pVal + dVal;
		System.out.print(pVal + " ++++ " + dVal);
		lastError = heading;
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

	public static final double kDefaultQuickStopThreshold = 0.2;
	public static final double kDefaultQuickStopAlpha = 0.1;

	private static double m_quickStopThreshold = kDefaultQuickStopThreshold;
	private static double m_quickStopAlpha = kDefaultQuickStopAlpha;
	private static double m_quickStopAccumulator = 0.0;
	private static double m_rightSideInvertMultiplier = -1.0;

	public static Tuple curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		xSpeed = Math.max(-1.0, Math.min(1.0, xSpeed));

		zRotation = Math.max(-1.0, Math.min(1.0, zRotation));

		double angularPower;
		boolean overPower;

		if (isQuickTurn) {
			if (Math.abs(xSpeed) < m_quickStopThreshold) {
				m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator + m_quickStopAlpha * zRotation * 2;
			}
			overPower = true;
			angularPower = zRotation;
		} else {
			overPower = false;
			angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

			if (m_quickStopAccumulator > 1) {
				m_quickStopAccumulator -= 1;
			} else if (m_quickStopAccumulator < -1) {
				m_quickStopAccumulator += 1;
			} else {
				m_quickStopAccumulator = 0.0;
			}
		}

		double leftMotorOutput = xSpeed + angularPower;
		double rightMotorOutput = xSpeed - angularPower;

		// If rotation is overpowered, reduce both outputs to within acceptable range
		if (overPower) {
			if (leftMotorOutput > 1.0) {
				rightMotorOutput -= leftMotorOutput - 1.0;
				leftMotorOutput = 1.0;
			} else if (rightMotorOutput > 1.0) {
				leftMotorOutput -= rightMotorOutput - 1.0;
				rightMotorOutput = 1.0;
			} else if (leftMotorOutput < -1.0) {
				rightMotorOutput -= leftMotorOutput + 1.0;
				leftMotorOutput = -1.0;
			} else if (rightMotorOutput < -1.0) {
				leftMotorOutput -= rightMotorOutput + 1.0;
				rightMotorOutput = -1.0;
			}
		}

		// Normalize the wheel speeds
		double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
		if (maxMagnitude > 1.0) {
			leftMotorOutput /= maxMagnitude;
			rightMotorOutput /= maxMagnitude;
		}

		return new Tuple(leftMotorOutput, rightMotorOutput );
	}

	public static void main(String[] args) throws IOException {
		System.out.println(((-730.0%360.0)+360.0)%360.0);
	}
}
