package org.usfirst.frc.team3623.util;

public class Utils {

	
	public static Boolean threshold(double value, double goal, double epsilon) {
		return Math.abs(value-goal) <= epsilon;
	}
}
