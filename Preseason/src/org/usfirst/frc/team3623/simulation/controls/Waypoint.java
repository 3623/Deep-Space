package org.usfirst.frc.team3623.simulation.controls;

import org.usfirst.frc.team3623.robot.util.Pose;

public class Waypoint {
//	public final Pose point;
	public final double kRadius;
	public final double kLookAhead;
	public final double x, y, heading;
	
	public Waypoint(Pose point) {
		this.x = point.x;
		this.y = point.y;
		this.heading = point.heading;
		this.kRadius = 0.0;
		this.kLookAhead = 0.0;
	}
	
	public Waypoint(Pose point, double radius, double lookAhead) {
		this.x = point.x;
		this.y = point.y;
		this.heading = point.heading;
		this.kRadius = radius;
		this.kLookAhead = lookAhead;
	}
	
	public Waypoint(double x, double y, double heading, double radius, double lookAhead) {
		this.x = x;
		this.y = y;
		this.heading = heading;
		this.kRadius = radius;
		this.kLookAhead = lookAhead;
	}
	
	public Waypoint(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
		this.kRadius = 0.0;
		this.kLookAhead = 0.0;
	}
	
}
