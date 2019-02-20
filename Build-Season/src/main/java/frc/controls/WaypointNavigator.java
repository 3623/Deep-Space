package frc.controls;

import java.util.ArrayList;

import frc.util.Pose;


public class WaypointNavigator { 
	private ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
	private int index = 1;
	private Waypoint curWaypoint;
	private Waypoint prevWaypoint;
	private final double kLookAheadDefault = 0.6;
	private double kLookAhead = kLookAheadDefault;
	private final double kRadiusDefault = 0.6;
	private double kRadius = kRadiusDefault;

	
	public void addWaypoint(Waypoint curWaypoint) {
		waypoints.add(curWaypoint);
	}
	
	public Pose updatePursuit(Pose position) {
		updateWaypoint();
		if (atWaypoint(curWaypoint, position, kRadius) && index != waypoints.size()-1) {
			index++;
//			updateWaypoint();
		}
		

		double dist = Math.sqrt(Math.pow(curWaypoint.x - position.x, 2) + Math.pow(curWaypoint.y - position.y, 2));
		
		double pathAngle = Math.atan2(curWaypoint.x-prevWaypoint.x, curWaypoint.y-prevWaypoint.y);
		double stateAngle = Math.atan2(curWaypoint.x-position.x, curWaypoint.y-position.y);
		double relativeAngle = stateAngle - pathAngle;
		double lineDist = dist*Math.cos(relativeAngle);
		if (lineDist > kLookAhead) {
			lineDist -= kLookAhead;
		} else {
			lineDist = 0;
		}
		
		double xChase = curWaypoint.x - (lineDist * Math.sin(pathAngle));
		double yChase = curWaypoint.y - (lineDist * Math.cos(pathAngle));
		double chaseRelativeAngle = Math.atan2(xChase-position.x, yChase-position.y);
		double chaseAngle = pathAngle + (chaseRelativeAngle) - position.heading;
		Pose pursuit = new Pose(xChase, yChase, Math.toDegrees(chaseAngle));
		
//		System.out.println(Math.toDegrees());
//		System.out.println(Math.toDegrees(chaseAngle) + " = " + 
//											Math.toDegrees(chaseRelativeAngle) + " - " + 
//											Math.toDegrees(position.heading) );
		return pursuit;
	}

	private void updateWaypoint() {
		prevWaypoint = waypoints.get(index-1);
		curWaypoint = waypoints.get(index);
		if (curWaypoint.kRadius!=0.0) kRadius = curWaypoint.kRadius;
		else kRadius = kRadiusDefault;
		if (curWaypoint.kLookAhead!=0.0) kLookAhead = curWaypoint.kLookAhead;
		else kLookAhead = kLookAheadDefault;
	}
	
	private Boolean atWaypoint(Waypoint curWaypoint, Pose position, double radius) {
		double dist = Math.sqrt(Math.pow(curWaypoint.x - position.x, 2) + Math.pow(curWaypoint.y - position.y, 2));
		return dist < radius;
	}
}