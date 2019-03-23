package frc.util;

public class Utils {

	
	public static Boolean withinThreshold(double value, double goal, double epsilon) {
		return Math.abs(value-goal) <= epsilon;
	}

	public static Boolean outsideDeadband(double value, double center, double deadband) {
		return Math.abs(value-center) >= deadband;
	}
}
