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
    private static final double MAX_SPEED = 3.3;
    private static final double UPDATE_RATE = 200.0;

    private ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
    private Waypoint curWaypoint;
    private int index = 0;

    private Boolean isFinished = false;

    private static double kRadiusPath = 0.0;
    private static final double kRadiusCritical = 0.1;
    private static final double kEpsilonPath = 5.0;
    private static final double kEpsilonCritical = 3.0;
    private static final double kV = 1.0 / 14.0;
    private static final double kTurn = 1.5 / 80.0;
    private static final double kMaxSplineAngle = Math.PI * 0.3;

    private Pose robotPose;

    double a = 0;
    double b = 0;

    double feedForwardSpeed = 0.0;
    double distanceFromWaypoint = 100.0;

    Boolean debug = false;

    /**
     * Updates the path follower with a new robot pose. Should be called at rate
     * equal to {@code UPDATE_RATE}.
     * 
     * @param robotPose the current robot pose, with position and velocities
     * @return a tuple with left and right wheel voltages
     */
    public Tuple updatePursuit(Pose robotPose) {
        curWaypoint = waypoints.get(index);
        this.robotPose = robotPose;

        feedForwardSpeed = curWaypoint.kSpeed;
        debug = false;
        if (curWaypoint.isCritical) { // important to be at exactly

            if (distanceFromWaypoint < Math.abs(feedForwardSpeed) * 1.2) {
                // speed reduces as distance gets smaller
                feedForwardSpeed = Math.copySign(distanceFromWaypoint / 1.2, feedForwardSpeed);
                if (Math.abs(feedForwardSpeed) < 0.25) {
                    feedForwardSpeed = Math.copySign(0.25, feedForwardSpeed);
                }
            }
            if (atWaypoint(kRadiusCritical) || isFinished) {
                debug = true;
                feedForwardSpeed = 0.0;
                if (atHeading(kEpsilonCritical)) {
                    // at point and heading, we're done
                    if (!isFinished)
                        System.out.println("At Waypoint: " + index + " (" + curWaypoint.toString() + ")");
                    if (index == waypoints.size() - 1 || isFinished) {
                        if (!isFinished)
                            System.out.println("Finished Path Following");
                        isFinished = true;
                        return new Tuple(0.0, 0.0);
                    } else {
                        index++;
                        curWaypoint = waypoints.get(index);
                    }

                } else {
                    // at point but not heading, just turn to the point
                    double ptrOutput = DrivetrainControls.turnToAngle(curWaypoint.heading, robotPose.heading);
                    return DrivetrainControls.curvatureDrive(0.0, ptrOutput, true);
                }
            }
        } else if (atWaypoint(kRadiusPath) && atHeading(kEpsilonPath)) {
            // at non-critical waypoint
            System.out.println("At Waypoint: " + index + " (" + curWaypoint.toString() + ")");
            index++;
            curWaypoint = waypoints.get(index);
            debug = true;
        }
        // if not in a special case, just run path following
        return pathFollowing(robotPose);
    }

    /**
     * Uses a cubic spline calculated OTF to figure out a projected change in angle
     * required to follow path and uses this as a feed forward value in conjuction
     * with a d term used to cancel out rotational inertia of the robot. This method
     * cheats by setting the initial point of the cubic spline as x=0, y=0, dx=0 to
     * make calculations simpler. This means that the waypoint has to be converted
     * to local coordinates in reference to the robot.
     * 
     * @return a tuple of left and right output voltages
     */
    public Tuple pathFollowing(Pose robotPose) {
        getPathGeometry(robotPose, curWaypoint);
        double nextSpeed = ((MAX_SPEED * feedForwardSpeed) * 0.1) + (robotPose.velocity * 0.9);
        double deltaX = nextSpeed / UPDATE_RATE;
        if (Math.signum(deltaX) != Math.signum(feedForwardSpeed))
            deltaX = 0.0;
        /*
         * Average of ffSpeed and actual speed scaled by cosine (to account for how far
         * off straight the robot has to drive) and cos again (the further off straight
         * the longer the curve) then divided by update rate (to get deltaX, the
         * position along the spline the robot will be at for the next update, giving a
         * feed forward point). If this just used actual speed, a stopped robot would
         * not look ahead.
         */

        if (deltaX != 0.0) {
            double y2 = (a * deltaX * deltaX * deltaX) + (b * deltaX * deltaX);
            double hypot = Geometry.hypotenuse(deltaX, y2);
            double ratio = Math.abs(deltaX / hypot);
            deltaX *= ratio;
        }

        kRadiusPath = Math.abs(deltaX) * UPDATE_RATE * 0.1;
        double dx2 = (3.0 * a * deltaX * deltaX) + (2.0 * b * deltaX);
        double relativeFeedForwardAngle = Math.atan(dx2);
        /*
         * It turns out that tan and atan are relatively interchangeable here, but in
         * this case, atan is the actually correct function (convert from ratio to
         * angle) and works more accurately, different from above usage, where atan is
         * the incorrect function but works more elegantly
         */

        if (false) {
            System.out.println(relativeAdjacDist + " " + relativeOpposDist + " " + relativeGoalDeriv);
            System.out.println(a + " " + b + " " + deltaX);
        }

        double turnOutput = -Math.toDegrees(relativeFeedForwardAngle) * kTurn;
        double outputLeft = ((feedForwardSpeed * kV) + turnOutput) * 12.0;
        double outputRight = ((feedForwardSpeed * kV) - turnOutput) * 12.0;

        return new Tuple(outputLeft, outputRight);
    }

    private Tuple getPathGeometry(Pose startPoint, Pose goalPoint) {
        calculateDistanceFromWaypoint();

        double straightPathAngle = Math.atan2(goalPoint.x - startPoint.x, goalPoint.y - startPoint.y);
        double relativeAngle = startPoint.r - straightPathAngle;
        double relativeOpposDist = distanceFromWaypoint * Math.sin(relativeAngle);
        double relativeAdjacDist = distanceFromWaypoint * Math.cos(relativeAngle);
        double relativeGoalAngle = startPoint.r - goalPoint.r;
        relativeGoalAngle = Utils.limit(relativeGoalAngle, kMaxSplineAngle,
        -kMaxSplineAngle);
        double relativeGoalDeriv = Math.tan(relativeGoalAngle);

        return generateSpline(relativeAdjacDist, relativeOpposDist, relativeGoalDeriv);
    }
    /**
     * Calculates the value of two coefficients (a & b) of a cubic spline specified
     * by two points and derivatives.
     * 
     * @Note The first point is assumed to be (0, 0) with a derivative of 0. Second
     *       point must be in reference to this point
     * @param x  the x coordinate of the second point
     * @param y  the y coordinate of the second point
     * @param dx the desired slope of the second point
     * @implNote Not complicated, just two equations derived from solving the system
     *           of equations where x1=0, y1=0, and dx1=0, and x2, y2, and dx2 are
     *           specified in relation to p1, and y=ax^3+bx^2+cx+d (c and d are
     *           equal to 0 because of definition)
     */
    private static Tuple generateSpline(double x, double y, double dx) {
        a = ((x * dx) - (2 * y)) / (x * x * x);
        b = ((3 * y) - (dx * x)) / (x * x);
        return new Tuple(a, b);
    }
    /**
     * Calculates euclidean distance between robot pose and current waypoint.
     * Updates the {@code distanceFromWaypoint} value
     */
    private void calculateDistanceFromWaypoint() {
        distanceFromWaypoint = Geometry.distance(curWaypoint.x, pose.x, curWaypoint.y, pose.y);
    }

    /**
     * Helper function to check if the robot is within a radius of the desired
     * waypoint
     * 
     * @param radius the desired radius of the robot to the point
     * @return true if the robot is within the desired radius of the point
     */
    private Boolean atWaypoint(double radius) {
        // System.out.println(distanceFromWaypoint);
        return (distanceFromWaypoint < radius);
    }

    /**
     * Helper function to check if the robot heading is within a deadband of the
     * desired heading
     * 
     * @param epsilon the tolerance, in degrees, of the robots heading versus the
     *                desired
     * @return true if the robot is pointing at an angle within the desired epsilon
     *         of the waypoint
     */
    private Boolean atHeading(double epsilon) {
        // System.out.println(pose.heading + " " + curWaypoint.heading);
        return Utils.withinThreshold(pose.heading, curWaypoint.heading, epsilon);
    }

    /**
     * Checks whether or not the robot has finished following the path specified by
     * given waypoints
     * 
     * @return true if the robot has finished the path specified
     */
    public Boolean getIsFinished() {
        return isFinished;
    }

    /**
     * Clears the array list of waypoints and resets index so that the path follower
     * can be used again
     */
    public void clearWaypoints() {
        waypoints.clear();
        index = 0;
        isFinished = false;
    }

    /**
     * Returns the current waypoint being followed by the path follower
     * 
     * @return {@link Waypoint}
     */
    public Waypoint getCurrentWaypoint() {
        return curWaypoint;
    }

    /**
     * Adds a waypoint to the list of waypoints (FILO)
     * 
     * @param newWaypoint see {@link Waypoint}
     */
    public void addWaypoint(Waypoint newWaypoint) {
        waypoints.add(newWaypoint);
    }

    /**
     * Contains information to define a point along a desired path
     */
    public static class Waypoint extends Pose {
        // public final Pose point;
        protected double kSpeed;
        protected Boolean isCritical;

        /**
         * Constructor for waypoint
         * 
         * @param x        in meters
         * @param y        in meters
         * @param heading  in degrees. Call .r for radians
         * @param speed    in desired speed on a scale of -1 to 1
         * @param critical whether or not the waypoint is critical. Will stop at a
         *                 critical waypoint
         */
        public Waypoint(double x, double y, double heading, double speed, Boolean critical) {
            super(x, y, heading);
            this.kSpeed = speed;
            this.isCritical = critical;
        }

        public Waypoint(double x, double y, double heading) {
            this(x, y, heading, 0.0, false);
        }

        public Waypoint(Pose pose) {
            this(pose.x, pose.y, pose.heading, 0.0, false);

        }

        public Waypoint(double x, double y, double heading, double speed) {
            this(x, y, heading, speed, false);
        }
        
        @Override
        public String toString() {
            return "x: " + x + ", y: " + y + ", heading: " + heading + ", speed: " + kSpeed;
        }
    }

    public static void main(String[] args) {
    }
}
