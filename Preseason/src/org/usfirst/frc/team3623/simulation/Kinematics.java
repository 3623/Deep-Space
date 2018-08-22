package org.usfirst.frc.team3623.simulation;

public class Kinematics {

	public double torqueToForce(double torque, double radius) {
		double force = torque / radius;
		return force;
	}
	
	/**
	 * The radius of the robot about the Instantaneous Center of Curvature (ICC)
	 * Used to infinitesimally calculate the displacement of the robot
	 * A positive radius is to the right of the robot (relative to robot) and negative is left
	 * @see <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">
	 * Columbia University: CS W4733 NOTES - Differential Drive Robots</a>
	 * @param wheelBase width between left and right sides of the drivetrain, meters
	 * @param left velocity of the left wheel, m/s
	 * @param right velocity of the right wheel, m/s
	 * @return the radius from the center of the robot to the ICC
	 */
	public static double radiusICC(double wheelBase, double left, double right) {
		return -(wheelBase/2)*(left+right)/(right-left);
	}
	
	/**
	 * The angular velocity of the robot about the Instantaneous Center of Curvature (ICC)
	 * Used to infinitesimally calculate the displacement of the robot
	 * A positive radius is to the left of the robot (relative to robot) and negative is right
	 * @see <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">
	 * Columbia University: CS W4733 NOTES - Differential Drive Robots</a>
	 * @param wheelBase width between left and right sides of the drivetrain, meters
	 * @param left velocity of the left wheel, m/s
	 * @param right velocity of the right wheel, m/s
	 * @return
	 */
	public static double velocityICC(double wheelBase, double left, double right) {
		return (right-left)/wheelBase;
	}
}
