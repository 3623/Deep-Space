/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controls;

import java.util.ArrayList;

import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Tuple;
import frc.util.Utils;

/**
 * Add your docs here.
 */
public class CubicSplineFollower {
    private static final double ROTATION_RATE = 60.0;
    private static final double MAX_SPEED = 3.0;
    private static final double SAMPLE_RATE = 50.0;

    private ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
    private Waypoint curWaypoint;
    private int index = 0;

    private Boolean isFinished = false;

    private static double kRadiusPath = 0.4;
    private static final double kRadiusCritical = 0.7;
    private static final double kRadiusFinal = 0.05;
    private static final double kEpsilonPath = 5.0;
    private static final double kEpsilonFinal = 1.0;
    private static final double kD = 0.5;
    private static final double kFF = 1.0/ROTATION_RATE*4.5;

    private Pose pose;

    double a = 0;
    double b = 0;

    double feedForwardSpeed = 0.0;
    double distanceFromWaypoint = 0.0;

    public Tuple updatePursuit(Pose robotPose) {
        curWaypoint = waypoints.get(index);
        pose = robotPose;
  
        calculateDistanceFromWaypoint();

        feedForwardSpeed = curWaypoint.kSpeed;
        if (index == waypoints.size() - 1) { 
            // Last waypoint, important to be at exactly
            if(distanceFromWaypoint < feedForwardSpeed) 
                // speed reduces as distance gets smaller
                 feedForwardSpeed = distanceFromWaypoint;
            if (atWaypoint(kRadiusFinal) || isFinished){
                feedForwardSpeed = 0.0;
                if(atHeading(kEpsilonFinal)){ 
                    // at point and heading, we're done
                    if (isFinished != true){ //print that we're there only once
                        System.out.println("At Waypoint Final: (" + curWaypoint.toString() + ")");
            index++;
                    }
                    isFinished = true;
                    return new Tuple (0.0, 0.0);
                } else { 
                    // at point but not heading, just turn to the point
                    double ptrOutput = DrivetrainControls.turnToAngle(curWaypoint.heading, pose.heading);
                    return DrivetrainControls.curvatureDrive(0.0, ptrOutput, true); 
                }
            } else if (atWaypoint(kRadiusCritical)){ 
                // need to switch algorithms to one which gets to the point more urgently
                // return toPoint();
            }
        } else if (atWaypoint(kRadiusPath) && atHeading(kEpsilonPath)) { 
            // at non-critical waypoint
            System.out.println("At Waypoint: " + index + " (" + curWaypoint.toString() + ")");
            index++;
            curWaypoint = waypoints.get(index);
        } 
        // if not in a special case, just run path following
        return pathFollowing();
    }

    public Tuple pathFollowing(){
        double straightPathAngle = Math.atan2(curWaypoint.x - pose.x, curWaypoint.y - pose.y);
        double relativeAngle = pose.r - straightPathAngle;
        double relativeOpposDist = distanceFromWaypoint * Math.sin(relativeAngle);
        double relativeAdjacDist = distanceFromWaypoint * Math.cos(relativeAngle);
        double relativeGoalAngle = pose.r - curWaypoint.r;
        // relativeGoalAngle = Utils.limit(relativeGoalAngle, Math.PI/3.0, -Math.PI/3.0);
        double relativeGoalDeriv = Math.atan(relativeGoalAngle);

        generateSpline(relativeAdjacDist, relativeOpposDist, relativeGoalDeriv);

        double deltaX = ((MAX_SPEED * feedForwardSpeed) + pose.velocity)/2.0 / SAMPLE_RATE;
        double y2 = (a * deltaX * deltaX * deltaX) + (b * deltaX * deltaX);
        double dx2 = (3.0 * a * deltaX * deltaX) + (2.0 * b * deltaX);
        double relativeFeedForwardAngle = Math.atan(dx2);

        double rotationSpeedFF = -Math.toDegrees(relativeFeedForwardAngle)%360.0*kFF;
        double rotationSpeedD = -pose.angularVelocity/360.0*kD;
        double rotationSpeed = rotationSpeedFF + rotationSpeedD;
        return DrivetrainControls.curvatureDrive(feedForwardSpeed, rotationSpeed, true);
    }

    public Tuple toPoint(){
        double pathAngle = curWaypoint.r;
        double stateAngle = Math.atan2(curWaypoint.x-pose.x, curWaypoint.y-pose.y);
        
		double xChase = curWaypoint.x;
		double yChase = curWaypoint.y;
		double chaseRelativeAngle = Math.atan2(xChase-pose.x, yChase-pose.y);
        double chaseAngle = (pathAngle - pose.r) + (chaseRelativeAngle - pathAngle);
        double error = Math.toDegrees(chaseAngle);

        double rotationSpeedFF = error%360.0*kFF;
        double rotationSpeedD = -pose.angularVelocity/360.0*kD;
        double rotationSpeed = rotationSpeedFF + rotationSpeedD;
        return DrivetrainControls.curvatureDrive(feedForwardSpeed, rotationSpeed, true);
    }

    public void generateSpline(double x, double y, double dx) {
        this.a = ((x * y) - (2 * y)) / (x * x * x);
        this.b = ((3 * y) - (dx * x)) / (x * x);
    }

    private void calculateDistanceFromWaypoint(){
        distanceFromWaypoint = Geometry.distance(curWaypoint.x, pose.x, curWaypoint.y, pose.y);
    }

    private Boolean atWaypoint(double radius) {
        return (distanceFromWaypoint < radius);
    }

    private Boolean atHeading(double epsilon){
        return Utils.withinThreshold(pose.heading, curWaypoint.heading, epsilon);
    }

    public Boolean getIsFinished() {
        return isFinished;
    }

    public void clearWaypoints() {
        waypoints.clear();
        index = 0;
        isFinished = false;
    }

    public Waypoint getCurrentWaypoint() {
        return curWaypoint;
    }

    public void addWaypoint(Waypoint curWaypoint) {
        waypoints.add(curWaypoint);
    }

    public static class Waypoint {
        // public final Pose point;
        protected double kSpeed;
        public double x, y, r, heading;

        /**
         * Constructor for waypoint without driving params
         * 
         * @param x,       in meters
         * @param y,       in meters,
         * @param heading, in degrees. Call .r for radians
         */
        public Waypoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.r = Math.toRadians(heading);
            this.heading = heading;
            this.kSpeed = 0.0;
        }

        public Waypoint(Pose pose) {
            this.x = pose.x;
            this.y = pose.y;
            this.r = pose.r;
            this.heading = pose.heading;
            this.kSpeed = 0.0;
        }

        public Waypoint(double x, double y, double heading, double speed) {
            this.x = x;
            this.y = y;
            this.r = Math.toRadians(heading);
            this.heading = heading;
            this.kSpeed = speed;
        }

        public String toString(){
            return "x: " + x + ", y: " + y + ", heading: " + heading + ", speed" + kSpeed;
        }
    }

    public static void main(String[] args) {
    }
}
