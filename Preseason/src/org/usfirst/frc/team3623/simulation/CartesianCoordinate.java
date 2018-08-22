package org.usfirst.frc.team3623.simulation;

/**
 * Simple Cartesian coordinate class
 * 
 * @author eric
 *
 */
public class CartesianCoordinate {
	double x;
	double y;
	double heading;
	
	public CartesianCoordinate(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
	}
	
	public CartesianCoordinate(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public CartesianCoordinate() {
		this.x = 0.0;
		this.y = 0.0;
		this.heading = 0.0;
	}
	
	public void update(double deltaX, double deltaY) {
		this.x += deltaX;
		this.y += deltaY;
	}
}
