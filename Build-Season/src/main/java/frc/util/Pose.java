package frc.util;

/**
 * Simple Cartesian coordinate class
 * 
 * @author eric
 *
 */
public class Pose {
	public double x;
	public double y;
	public double r;
	public double heading;
	
	/**
	 * Constructor for CartesianCoordinate class
	 * @param x the x location in meters
	 * @param y the y location in meters
	 * @param heading the heading angle in degress, is converted into radians
	 */
	public Pose(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.r = Math.toRadians(heading);
		this.heading = heading;
	}
	
	public Pose(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public Pose() {
		this.x = 0.0;
		this.y = 0.0;
		this.heading = 0.0;
		this.r = 0.0;
	}

	public void update(double deltaX, double deltaY, double deltaR) {
		this.x += deltaX;
		this.y += deltaY;
		this.r += deltaR;
		this.heading = Math.toDegrees(r);
	}

	public void setHeading(double heading) {
		this.heading = heading;
		this.r = Math.toRadians(heading);
	}
}
