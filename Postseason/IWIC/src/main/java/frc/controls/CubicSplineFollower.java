/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controls;

import frc.util.Pose;

/**
 * Add your docs here.
 */
public class CubicSplineFollower {
    private static final double SAMPLE_RATE = 50.0;
    double a = 0;
    double b = 0;

    public void generateSpline(double x, double y, double dx){
        this.a = ((x*y) - (2*y))/(x*x*x);
        this. b = ((3*y)-(dx*x))/(x*x);
    }

    public Pose feedForward(double deltaX){
        double y2 = (a*deltaX*deltaX*deltaX) + (b*deltaX*deltaX);
        double dx2 = (3.0*a*deltaX*deltaX) + (2.0*b*deltaX);
        double relativeAngle = Math.tan(dx2);
        return new Pose(deltaX, y2, Math.toDegrees(relativeAngle));
    }

    public double stuff(Pose startPose, Pose goalPose){
        double dist = Math.sqrt((startPose.x-goalPose.x)*(startPose.x-goalPose.x) + (startPose.y-goalPose.y)*(startPose.y-goalPose.y));
        double straightPathAngle = Math.atan2(startPose.x-goalPose.x, startPose.y-goalPose.y);
        double relativeAngle = startPose.r - straightPathAngle;
        double relativeOpposDist = dist*Math.sin(relativeAngle);
        double relativeAdjacDist = dist*Math.cos(relativeAngle);
        double relativeGoalAngle = startPose.r-goalPose.r;
        double relativeGoalDeriv = Math.atan(relativeGoalAngle);

        generateSpline(relativeAdjacDist, relativeOpposDist, relativeGoalDeriv);

        Pose relativeFeedForwardPose = feedForward(1.8/SAMPLE_RATE);
        double feedForwardAngle = startPose.r + relativeFeedForwardPose.r;
        // System.out.println(Math.toDegrees(feedForwardAngle));
        return Math.toDegrees(feedForwardAngle);
    }


	public static void main ( String[] args ) {
        CubicSplineFollower cubes = new CubicSplineFollower();
        Pose pose = new Pose(1.0, 0.0, 45.0);
        Pose startPoint = new Pose(0.0, 0.0, 45.0);
        Pose goalPoint = new Pose(1.0, 1.0, 0.0);
        double goalAngle = cubes.stuff(startPoint, goalPoint);
      }
}
