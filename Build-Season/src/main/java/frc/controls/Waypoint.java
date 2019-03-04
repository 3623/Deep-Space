package frc.controls;

import frc.util.Pose;

public class Waypoint {
//	public final Pose point;
	protected final double kLookAhead;
	protected final double kSpeedFactor;
	protected final double kRadius;
	public double x, y, r, heading;
	protected final Boolean driveBackwards;
	
	public Waypoint(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.r = Math.toRadians(heading);
		this.heading = heading;
		this.kRadius = 0.0;
		this.kLookAhead = 0.0;
		this.kSpeedFactor = 0.0;
		this.driveBackwards = false;
	}

	public Waypoint(double x, double y, double heading, double speed, double lookAhead, double radius, Boolean inverted) {
		this.x = x;
		this.y = y;
		this.r = Math.toRadians(heading);
		this.heading = heading;
		this.kSpeedFactor = speed/lookAhead;
		this.kLookAhead = lookAhead;
		this.kRadius = radius;
		this.driveBackwards = inverted;
	}
	
}
