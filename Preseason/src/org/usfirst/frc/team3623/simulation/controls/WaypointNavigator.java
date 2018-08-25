package org.usfirst.frc.team3623.simulation.controls;

import java.util.ArrayList;

import org.usfirst.frc.team3623.robot.util.Pose;


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
	
	public void updateWaypoint() {
		prevWaypoint = waypoints.get(index-1);
		curWaypoint = waypoints.get(index);
		if (curWaypoint.kRadius!=0.0) kRadius = curWaypoint.kRadius;
		else kRadius = kRadiusDefault;
		if (curWaypoint.kLookAhead!=0.0) kLookAhead = curWaypoint.kLookAhead;
		else kLookAhead = kLookAheadDefault;
	}
	
	public Pose updatePursuit(Pose position) {
		updateWaypoint();
		if (atWaypoint(curWaypoint, position, kRadius) && index != waypoints.size()-1) {
			index++;
//			updateWaypoint();
		}
		

		double dist = Math.sqrt(Math.pow(curWaypoint.x - position.x, 2) + Math.pow(curWaypoint.y - position.y, 2));
		
		
		double lineAngle = Math.atan2(-prevWaypoint.x+curWaypoint.x, -prevWaypoint.y+curWaypoint.y);
		double stateAngle = Math.atan2(-curWaypoint.x+position.x, -curWaypoint.y+position.y);
		double relativeAngle = Math.PI + stateAngle - lineAngle;
		double lineDist = dist*Math.cos(relativeAngle);
//		anglePursuit = relativeAngle - position.heading;
		if (lineDist > kLookAhead) {
			lineDist -= kLookAhead;
		} else {
			lineDist = 0;
//			anglePursuit -= Math.PI;
		}
		double xPursuit = curWaypoint.x - (lineDist * Math.sin(lineAngle));
		double yPursuit = curWaypoint.y - (lineDist * Math.cos(lineAngle));
		double pursuitRelativeAngle = Math.atan2(xPursuit-position.x, yPursuit-position.y);
		double anglePursuit = lineAngle + (pursuitRelativeAngle) - position.heading;
		Pose pursuit = new Pose(xPursuit,
																yPursuit,
																Math.toDegrees(anglePursuit));
//		System.out.println(Math.toDegrees());
		System.out.println(Math.toDegrees(anglePursuit) + " = " + 
											Math.toDegrees(pursuitRelativeAngle) + " - " + 
											Math.toDegrees(position.heading) );
		return pursuit;
	}
	
	private Boolean atWaypoint(Waypoint curWaypoint, Pose position, double radius) {
		double dist = Math.sqrt(Math.pow(curWaypoint.x - position.x, 2) + Math.pow(curWaypoint.y - position.y, 2));
		return dist < radius;
	}
	
//	 ============== Main Navigation Controller =================

//		        state = self.state
//		        # print 'Position: x: %.1f y: %.1f yaw %.1f' %(state.x, state.y, state.yaw)
//		        if (self.goal_index == (len(self.goal_list)-1)) and self.lock == 0:
//					self.goal_index = self.goal_index
//					print "Goal Posistion Last:  X %.2f   Y %.2f   Z %.2f   Yaw %.2f" %(self.position_goal[0], self.position_goal[1], self.position_goal[2], self.position_goal[3])
//					self.lock = 1
//		        elif self.at_goal(self.position_goal) and self.lock == 0:
//					self.goal_index += 1
//					self.position_goal = self.goal_list[self.goal_index][0]
//					self.position_gain = self.goal_list[self.goal_index][1]
//					print "Goal Posistion %.0f:  X %.2f   Y %.2f   Z %.2f   Yaw %.2f" %(self.goal_index, self.position_goal[0], self.position_goal[1], self.position_goal[2], self.position_goal[3])
//		        
//				#print self.goal_index-1, len(self.goal_list)
//		        
//
//				self.publish_goal.publish(self.make_twist_msg(x_goal,y_goal,z,yaw))
//				self.publish_gain.publish(np.array(self.position_gain, dtype=np.float32))
//
//
//		    def at_goal(self, goal):
//		        # Calculate the error between where you are and the position goal
//		        goal = goal  # Twist Message
//		        state = self.state
//		        # print dist_to_goal(goal)
//		        angle = state.yaw * 3.1415 / 180.0
//		        err_xy = math.sqrt((goal[0] - state.x) ** 2 + (goal[1] - state.y) ** 2)
//		        err_z = abs(goal[2] - state.z)
//		        err_yaw = abs(goal[3] - state.yaw)
//
//		        # Determine if you are closer than self.XY_WP_THRES
//		        if (err_xy < self.XY_GOAL_THRES and err_z < self.Z_GOAL_THRES):
//		            return True
//		        else:
//		            return False
	 

}
