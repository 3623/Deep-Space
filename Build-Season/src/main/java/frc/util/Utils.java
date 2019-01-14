package frc.util;

public class Utils {

	
	public static Boolean threshold(double value, double goal, double epsilon) {
		return Math.abs(value-goal) <= epsilon;
	}
}
