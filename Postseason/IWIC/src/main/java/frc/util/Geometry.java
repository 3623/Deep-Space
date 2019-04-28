package frc.util;

public class Geometry {
		
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
		return distance(a.x, b.x, a.y, b.y);
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

				// System.out.println(Geometry.distance(left,  right));
				System.out.println(((180 - (-90))+360.0)%360.0);
	  }
}
