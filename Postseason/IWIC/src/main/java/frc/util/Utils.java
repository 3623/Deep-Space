package frc.util;

public class Utils {

	
	public static Boolean withinThreshold(double value, double goal, double epsilon) {
		return Math.abs(value-goal) <= epsilon;
	}

	public static Boolean outsideDeadband(double value, double center, double deadband) {
		return Math.abs(value-center) >= deadband;
	}

	public static double limit(double value, double upperBound, double lowerBound){
		return Math.max(lowerBound, Math.min(upperBound, value));
	}

	public static double limit(double value){
		return limit(value, 1.0, -1.0);
	}
}
