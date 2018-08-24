package org.usfirst.frc.team3623.simulation.controls;

import java.util.ArrayList;

import org.usfirst.frc.team3623.robot.util.CartesianCoordinate;

public class WaypointNavigator { 
	private ArrayList<CartesianCoordinate> waypoints = new ArrayList<CartesianCoordinate>();
	private int index = 1;
	private double kLookAhead = 0.9;
	
	public void addWaypoint(CartesianCoordinate waypoint) {
		waypoints.add(waypoint);
	}
	
	public CartesianCoordinate updatePursuit(CartesianCoordinate current) {
		if (atWaypoint(waypoints.get(index), current)) {
			index++;
		}
		
		CartesianCoordinate prevWaypoint = waypoints.get(index-1);
		CartesianCoordinate waypoint = waypoints.get(index);
		double dist = Math.sqrt(Math.pow(waypoint.x - current.x, 2) + Math.pow(waypoint.y - current.y, 2));
		
		
		double lineAngle = Math.atan2(-prevWaypoint.x+waypoint.x, -prevWaypoint.y+waypoint.y);
		double stateAngle = Math.atan2(-waypoint.x+current.x, -waypoint.y+current.y);
		double relativeAngle = Math.PI + stateAngle - lineAngle;
		double lineDist = dist*Math.cos(relativeAngle);
		double anglePursuit = lineAngle - Math.PI;
		if (lineDist > kLookAhead) {
			lineDist -= kLookAhead;
		} else {
			lineDist = 0;
//			anglePursuit -= Math.PI;
		}
		double xPursuit = waypoint.x - (lineDist * Math.sin(lineAngle));
		double yPursuit = waypoint.y - (lineDist * Math.cos(lineAngle));
		CartesianCoordinate pursuit = new CartesianCoordinate(xPursuit,
																yPursuit,
																Math.toDegrees(anglePursuit));
		System.out.println(Math.toDegrees(anglePursuit));

		
		return pursuit;
	}
	
	private Boolean atWaypoint(CartesianCoordinate waypoint, CartesianCoordinate position) {
		double dist = Math.sqrt(Math.pow(waypoint.x - position.x, 2) + Math.pow(waypoint.y - position.y, 2));
		return dist < kLookAhead;
	}
	
//	 ============== Main Navigation Controller =================

//		        state = self.state
//		        # print 'Position: x: %.1f y: %.1f yaw %.1f' %(state.x, state.y, state.yaw)
//		        if (self.goal_index == (len(self.goal_list)-1)) and self.lock == 0:
//					self.goal_index = self.goal_index
//					print "Goal Posistion Last:  X %.2f   Y %.2f   Z %.2f   Yaw %.2f" %(self.current_goal[0], self.current_goal[1], self.current_goal[2], self.current_goal[3])
//					self.lock = 1
//		        elif self.at_goal(self.current_goal) and self.lock == 0:
//					self.goal_index += 1
//					self.current_goal = self.goal_list[self.goal_index][0]
//					self.current_gain = self.goal_list[self.goal_index][1]
//					print "Goal Posistion %.0f:  X %.2f   Y %.2f   Z %.2f   Yaw %.2f" %(self.goal_index, self.current_goal[0], self.current_goal[1], self.current_goal[2], self.current_goal[3])
//		        
//				#print self.goal_index-1, len(self.goal_list)
//		        
//
//				self.publish_goal.publish(self.make_twist_msg(x_goal,y_goal,z,yaw))
//				self.publish_gain.publish(np.array(self.current_gain, dtype=np.float32))
//
//
//		    def at_goal(self, goal):
//		        # Calculate the error between where you are and the current goal
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
