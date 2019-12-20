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

    public Boolean isFinished = false;

    private static double kRadiusPath = 0.0;
    private static final double kRadiusCritical = 0.1;
    private static final double kAngularErrorPath = 5.0;
    private static final double kAngularErrorCritical = 8.0;
    private static final double kTurn = 12.0 / 450.0; // Volts/turn speed(deg/s)
    private static final double kMaxSplineAngle = Math.PI * 0.3;
    private static final double kMaxOutput = 13.0;

    double ffSpeed = 0.0;

    Boolean debug = false;

    /**
     * Updates the path follower with a new robot pose. Should be called at rate
     * equal to {@code UPDATE_RATE}.
     *
     * @param robotPose the current robot pose, with position and velocities
     * @return a tuple with left and right wheel voltages
     */
    public Tuple updatePursuit(Pose robotPose) {
        if (waypoints.size() < 1)
            return new Tuple(0.0, 0.0);
        curWaypoint = waypoints.get(index);
        double distanceFromWaypoint = Geometry.distance(robotPose, curWaypoint);
        ffSpeed = curWaypoint.kSpeed;
        debug = false;
        if (curWaypoint.isCritical) { // important to be at exactly

            if (distanceFromWaypoint < Math.abs(ffSpeed) * 1.2) {
                // speed reduces as distance gets smaller
                ffSpeed = Math.copySign(distanceFromWaypoint / 1.2, ffSpeed);
                if (Math.abs(ffSpeed) < 0.25) {
                    ffSpeed = Math.copySign(0.25, ffSpeed);
                }
            }
            if (distanceFromWaypoint < kRadiusCritical || isFinished) {
                debug = true;
                ffSpeed = 0.0;
                if (Utils.withinThreshold(robotPose.heading, curWaypoint.heading, kAngularErrorCritical)) {
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
                    return DrivetrainControls.curvatureDrive(0.0, ptrOutput, true).scale(12.0);
                }
            }
        } else if (distanceFromWaypoint < kRadiusPath
                && Utils.withinThreshold(robotPose.heading, curWaypoint.heading, kAngularErrorPath)) {
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
        Tuple pathCoefficients = getPathGeometry(robotPose, curWaypoint);
        double a = pathCoefficients.left;
        double b = pathCoefficients.right;
        double nextSpeed = ((MAX_SPEED * ffSpeed) * 0.1) + (robotPose.velocity * 0.9);
        double deltaX = nextSpeed / UPDATE_RATE;
        if (Math.signum(deltaX) != Math.signum(ffSpeed))
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
        double relativeFFAngle = Math.atan(dx2);
        // Convert from derivative to angle

        double turnOutput = -Math.toDegrees(relativeFFAngle) * kTurn * UPDATE_RATE;
        double turnLimitedFFSpeed = Math.copySign(Math.abs(ffSpeed) - Math.abs(turnOutput / 12.0), ffSpeed);
        double outputLeft = (turnLimitedFFSpeed * 12.0) + turnOutput;
        double outputRight = (turnLimitedFFSpeed * 12.0) - turnOutput;
        double limitedOutLeft = Utils.limit(outputLeft, kMaxOutput);
        double limitedOutRight = Utils.limit(outputRight, kMaxOutput);

        if (false) {
            System.out.println(pathCoefficients.toString());
        }
        if (limitedOutLeft != outputLeft)
            System.out.println("Limited left: " + outputLeft);
        if (limitedOutRight != outputRight)
            System.out.println("Limited left: " + outputRight);

        return new Tuple(limitedOutLeft, limitedOutRight);
    }

    private Tuple getPathGeometry(Pose startPoint, Pose goalPoint) {
        double distanceFromWaypoint = Geometry.distance(startPoint, goalPoint);
        double straightPathAngle = Math.atan2(goalPoint.x - startPoint.x, goalPoint.y - startPoint.y);
        double relativeAngle = startPoint.r - straightPathAngle;
        double relativeOpposDist = distanceFromWaypoint * Math.sin(relativeAngle);
        double relativeAdjacDist = distanceFromWaypoint * Math.cos(relativeAngle);
        double relativeGoalAngle = startPoint.r - goalPoint.r;
        relativeGoalAngle = Utils.limit(relativeGoalAngle, kMaxSplineAngle, -kMaxSplineAngle);
        double relativeGoalDeriv = Math.tan(relativeGoalAngle);
        if (false)
            System.out.println(relativeAdjacDist + " " + relativeOpposDist + " " + relativeGoalDeriv);
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
        double a = ((x * dx) - (2 * y)) / (x * x * x);
        double b = ((3 * y) - (dx * x)) / (x * x);
        return new Tuple(a, b);
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

    public void addWaypoint(double x, double y, double heading, double speed, Boolean isCritical) {
        waypoints.add(new Waypoint(x, y, heading, speed, isCritical));
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
