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
		this.heading = Math.toRadians(heading);
	}
	
	public Pose(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public Pose() {
		this.x = 0.0;
		this.y = 0.0;
		this.heading = 0.0;
	}
	
	public void update(double deltaX, double deltaY) {
		this.x += deltaX;
		this.y += deltaY;
	}
}
