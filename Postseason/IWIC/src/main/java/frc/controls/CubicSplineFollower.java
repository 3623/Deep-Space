/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controls;

import java.util.ArrayList;

import frc.util.Pose;
import frc.util.Tuple;

/**
 * Add your docs here.
 */
public class CubicSplineFollower {
    private static final double ROTATION_RATE =15.0;
    private ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
    private Waypoint curWaypoint;
    private int index = 0;
    private double kRadius = 0.5;

    private Boolean isFinished = false;

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

    public Tuple updatePursuit(Pose currentPose) {
        curWaypoint = waypoints.get(index);
        if (atWaypoint(curWaypoint, currentPose, kRadius)) {
            if (index == waypoints.size() - 1) {
                isFinished = true;
                curWaypoint.kSpeed = 0.0;
            } else {
                index++;
                curWaypoint = waypoints.get(index);
            }
        }

        double dist = Math.sqrt((currentPose.x - curWaypoint.x) * (currentPose.x - curWaypoint.x)
                + (currentPose.y - curWaypoint.y) * (currentPose.y - curWaypoint.y));
        double straightPathAngle = Math.atan2(currentPose.x - curWaypoint.x, currentPose.y - curWaypoint.y);
        double relativeAngle = currentPose.r - straightPathAngle;
        double relativeOpposDist = dist * Math.sin(relativeAngle);
        double relativeAdjacDist = dist * Math.cos(relativeAngle);
        double relativeGoalAngle = currentPose.r - curWaypoint.r;
        double relativeGoalDeriv = Math.atan(relativeGoalAngle);

        generateSpline(relativeAdjacDist, relativeOpposDist, relativeGoalDeriv);

        Pose relativeFeedForwardPose = feedForward(1.8 / SAMPLE_RATE);
        double feedForwardAngle = currentPose.r + relativeFeedForwardPose.r;
        double rotationSpeed = Math.toDegrees(relativeFeedForwardPose.r)/ROTATION_RATE;
        System.out.println(rotationSpeed);
        return DrivetrainControls.arcadeDrive(curWaypoint.kSpeed, rotationSpeed);
    }

    private Boolean atWaypoint(Waypoint curWaypoint, Pose currentPose, double radius) {
        double dist = Math
                .sqrt(Math.pow(curWaypoint.x - currentPose.x, 2) + Math.pow(curWaypoint.y - currentPose.y, 2));
        return dist < radius;
    }

    private static final double SAMPLE_RATE = 50.0;

    double a = 0;
    double b = 0;

    public void generateSpline(double x, double y, double dx) {
        this.a = ((x * y) - (2 * y)) / (x * x * x);
        this.b = ((3 * y) - (dx * x)) / (x * x);
    }

    public Pose feedForward(double deltaX) {
        double y2 = (a * deltaX * deltaX * deltaX) + (b * deltaX * deltaX);
        double dx2 = (3.0 * a * deltaX * deltaX) + (2.0 * b * deltaX);
        double relativeAngle = Math.tan(dx2);
        return new Pose(deltaX, y2, Math.toDegrees(relativeAngle));
    }

    public static class Waypoint {
        // public final Pose point;
        protected double kSpeed;
        protected final double kRadius;
        public double x, y, r, heading;
        protected final Boolean driveBackwards;

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
            this.kRadius = 0.0;
            this.kSpeed = 0.0;
            this.driveBackwards = false;
        }

        public Waypoint(Pose pose) {
            this.x = pose.x;
            this.y = pose.y;
            this.r = pose.r;
            this.heading = pose.heading;
            this.kRadius = 0.0;
            this.kSpeed = 0.0;
            this.driveBackwards = false;
        }

        public Waypoint(double x, double y, double heading, double speed, double radius, Boolean inverted) {
            this.x = x;
            this.y = y;
            this.r = Math.toRadians(heading);
            this.heading = heading;
            this.kSpeed = speed;
            this.kRadius = radius;
            this.driveBackwards = inverted;
        }
    }

    public static void main(String[] args) {
        CubicSplineFollower cubes = new CubicSplineFollower();
        Pose pose = new Pose(1.0, 0.0, 45.0);
        Pose startPoint = new Pose(0.0, 0.0, 45.0);
        Pose goalPoint = new Pose(1.0, 1.0, 0.0);
        // double goalAngle = cubes.updatePursuit(startPoint);
    }
}
