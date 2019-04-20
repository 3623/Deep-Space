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
    private static final double ROTATION_RATE = 15.0;
    private static final double MAX_SPEED = 4.0;
    private static final double SAMPLE_RATE = 50.0;

    private ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
    private Waypoint curWaypoint;
    private int index = 0;

    private Boolean isFinished = false;

    private double kRadiusPath = 0.5;
    private double kRadiusFinal = 0.1;
    private double kEpsilonPath = 20.0;
    private double kEpsilonFinal = 3.0;


    private Pose position;

    double a = 0;
    double b = 0;

    double feedForwardSpeed = 0.0;
    double distanceFromWaypoint = 0.0;

    public Tuple updatePursuit(Pose currentPose) {
        curWaypoint = waypoints.get(index);
        position = currentPose;

        calculateDistanceFromWaypoint();

        if (index == waypoints.size() - 1) {
            if (atWaypoint(kRadiusFinal)){
                feedForwardSpeed = 0.0;
                if(atHeading(kEpsilonFinal)){
                    isFinished = true;
                    return new Tuple (0.0, 0.0);
                } else {
                    double ptrOutput = DrivetrainControls.turnToAngle(curWaypoint.heading, position.heading);
                    return DrivetrainControls.curvatureDrive(0.0, ptrOutput, true); 
                }
            } else {
                feedForwardSpeed = distanceFromWaypoint;
                return pointToPoint();
            }
        } else if (atWaypoint(kRadiusPath) && atHeading(kEpsilonPath)) {
            index++;
            curWaypoint = waypoints.get(index);
        } 
        feedForwardSpeed = curWaypoint.kSpeed;
        return pathFollowing();
    }

    public Tuple pathFollowing(){
        double straightPathAngle = Math.atan2(position.x - curWaypoint.x, position.y - curWaypoint.y);
        double relativeAngle = position.r - straightPathAngle;
        double relativeOpposDist = distanceFromWaypoint * Math.sin(relativeAngle);
        double relativeAdjacDist = distanceFromWaypoint * Math.cos(relativeAngle);
        double relativeGoalAngle = position.r - curWaypoint.r;
        double relativeGoalDeriv = Math.atan(relativeGoalAngle);

        generateSpline(relativeAdjacDist, relativeOpposDist, relativeGoalDeriv);

        double deltaX = MAX_SPEED * feedForwardSpeed / SAMPLE_RATE;
        double y2 = (a * deltaX * deltaX * deltaX) + (b * deltaX * deltaX);
        double dx2 = (3.0 * a * deltaX * deltaX) + (2.0 * b * deltaX);
        double relativeFeedForwardAngle = Math.tan(dx2);

        Pose relativeFeedForwardPose = new Pose(deltaX, y2, Math.toDegrees(relativeFeedForwardAngle));

        double rotationSpeed = relativeFeedForwardPose.heading/ROTATION_RATE;
        System.out.println();

        return DrivetrainControls.curvatureDrive(feedForwardSpeed, rotationSpeed, false);
    }

    public Tuple pointToPoint(){
        double straightPathAngle = Math.atan2(position.x - curWaypoint.x, position.y - curWaypoint.y);
        double relativeAngle = position.r - straightPathAngle;
        double relativeOpposDist = distanceFromWaypoint * Math.sin(relativeAngle);
        double relativeAdjacDist = distanceFromWaypoint * Math.cos(relativeAngle);
        double relativeGoalAngle = position.r - curWaypoint.r;
        double relativeGoalDeriv = Math.tan(relativeGoalAngle);

        generateSpline(relativeAdjacDist, relativeOpposDist, relativeGoalDeriv);

        double deltaX = MAX_SPEED * feedForwardSpeed / SAMPLE_RATE;
        double y2 = (a * deltaX * deltaX * deltaX) + (b * deltaX * deltaX);
        double dx2 = (3.0 * a * deltaX * deltaX) + (2.0 * b * deltaX);
        double relativeFeedForwardAngle = Math.atan(dx2);

        Pose relativeFeedForwardPose = new Pose(deltaX, y2, Math.toDegrees(relativeFeedForwardAngle));

        double feedForwardAngle = position.r + relativeFeedForwardPose.r;
        double rotationSpeed = relativeFeedForwardPose.heading/ROTATION_RATE;
        System.out.println();

        return DrivetrainControls.curvatureDrive(feedForwardSpeed, rotationSpeed, false);
    }

    public void generateSpline(double x, double y, double dx) {
        this.a = ((x * y) - (2 * y)) / (x * x * x);
        this.b = ((3 * y) - (dx * x)) / (x * x);
    }

    private void calculateDistanceFromWaypoint(){
        distanceFromWaypoint = Geometry.distance(curWaypoint.x, position.x, curWaypoint.y, position.y);
    }

    private Boolean atWaypoint(double radius) {
        return (distanceFromWaypoint < radius);
    }

    private Boolean atHeading(double epsilon){
        return Utils.withinThreshold(position.heading, curWaypoint.heading, epsilon);
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
    }

    public static void main(String[] args) {
        System.out.println(-540%180);
    }
}
