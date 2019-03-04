package frc.util;

public class Geometry {
	
	public static Pose center(Pose left, Pose right) {
		
		double x = (left.x + right.x)/2;
		double y = (left.y + right.y)/2;
		
		double xDif = -(left.x-right.x);
		double yDif = left.y - right.y;
		double angle = Math.atan2(yDif, xDif);
		double r = angle;
		
		Pose coordinate = new Pose(x, y, Math.toDegrees(r));
		return coordinate;
	}
	
	public static Pose inverseCenterLeft(Pose center, double drivebase) {
		double x = center.x - drivebase/2.0 * Math.cos(center.r);
		double y = center.y + drivebase/2.0 * Math.sin(center.r);
		Pose left = new Pose(x, y);
		return left;
	}
	
	public static Pose inverseCenterRight(Pose center, double drivebase) {
		double x = center.x + drivebase/2.0 * Math.cos(center.r);
		double y = center.y - drivebase/2.0 * Math.sin(center.r);
		Pose right = new Pose(x, y);
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
	
	public static double distance(Pose a, Pose b) {
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
	      Pose left = new Pose(6, 5);
	      Pose right = new Pose(5, 4);

				System.out.println(Geometry.distance(left,  right));
				System.out.println(Math.cos(180));
	  }
}
