package org.usfirst.frc.team3623.robot.util;

public class Geometry {
	
	public static CartesianCoordinate center(CartesianCoordinate left, CartesianCoordinate right) {
		CartesianCoordinate coordinate = new CartesianCoordinate();
		coordinate.x = (left.x + right.x)/2;
		coordinate.y = (left.y + right.y)/2;
		
		double xDif = -(left.x-right.x);
		double yDif = left.y - right.y;
		double angle = Math.atan2(yDif, xDif);
		coordinate.heading = angle;
		
		return coordinate;
	}
	
	public static CartesianCoordinate inverseCenterLeft(CartesianCoordinate center, double drivebase) {
		double x = center.x - drivebase/2.0 * Math.cos(center.heading);
		double y = center.y + drivebase/2.0 * Math.sin(center.heading);
		CartesianCoordinate left = new CartesianCoordinate(x, y);
		return left;
	}
	
	public static CartesianCoordinate inverseCenterRight(CartesianCoordinate center, double drivebase) {
		double x = center.x + drivebase/2.0 * Math.cos(center.heading);
		double y = center.y - drivebase/2.0 * Math.sin(center.heading);
		CartesianCoordinate right = new CartesianCoordinate(x, y);
		return right;
	}
	
	/**
	 * Returns side b, calculated using law of sines
	 * @param a length of side a
	 * @param sinA value for sine of angle A, opposite of side a
	 * @param sinB value for sine of angle B, opposite of side b
	 * @return the length of side b
	 */
	public static double sideFromLawOfSines(double a, double sinA, double sinB) {
		return (a * sinB / sinA);
	}
	
	public static double distance(CartesianCoordinate a, CartesianCoordinate b) {
		double distance = distance(a.x, b.x, a.y, b.y);
		return distance;
	}
	
	public static double distance(double x1, double x2, double y1, double y2) {
		double deltaX = x1 - x2;
		double deltaY = y1 - y2;
		double dist = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
		return dist;
	}


	// Testing calculations
	public static void main ( String[] args )
	  {
	      Geometry test = new Geometry();
	      CartesianCoordinate left = new CartesianCoordinate(6, 5);
	      CartesianCoordinate right = new CartesianCoordinate(5, 4);

	      System.out.println(Geometry.distance(left,  right));
	  }
}
