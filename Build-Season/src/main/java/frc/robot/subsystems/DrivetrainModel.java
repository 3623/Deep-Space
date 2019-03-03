package frc.robot.subsystems;

import frc.simulation.motors.CIMMotor;
import frc.simulation.motors.Motor;
import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Model for a differential drivetrain, simulates mass and motor performance
 * Integrates acceleration over time for both sides of the drivetrain and uses velocity to 
 * calculate the instantaneous curvature of the robot, which is then integrated for position
 * @author eric
 *
 */
public class DrivetrainModel {

	private final double DRIVETRAIN_MASS = 63.5; // kg
	private final double WHEEL_BASE = 0.67; // meters
	private final double CENTER_MASS = 0.381; // from left wheel
	private Boolean COAST_MODE = false; 
	public Pose center;
	
	private DrivetrainSide left, right;
	static final double WHEEL_RADIUS = 0.0774; // meters
	private static final double CIMS_PER_SIDE = 2.0; // Minicim is 0.58
	private static final double GEAR_RATIO = 10.75/1.0; // Reduction
	private static final double DRIVETRAIN_FRICTION = 115;
	private static final double MAX_FORCE = 250.0;
	private static final double MAX_TORQUE = MAX_FORCE * WHEEL_RADIUS;

	
	public DrivetrainModel() {
		center = new Pose(0.0, 0.0, 0.0); // Initial robot position
		left = new DrivetrainSide(Geometry.inverseCenterLeft(center, WHEEL_BASE),
									DRIVETRAIN_MASS/2);
		right = new DrivetrainSide(Geometry.inverseCenterRight(center, WHEEL_BASE),
									DRIVETRAIN_MASS/2);	
		
		if (COAST_MODE) {
			left.coast = true;
			right.coast = true;
		} else {
			left.coast = false;
			right.coast = false;
		}
	}

	public void setPosition(double x, double y, double r){
		center = new Pose(x, y, r);
		left.position = Geometry.inverseCenterLeft(center, WHEEL_BASE);
		right.position = Geometry.inverseCenterRight(center, WHEEL_BASE);
		zeroSensors();
	}

	public void zeroSensors(){
		left.velocity = 0.0;
		right.velocity = 0.0;
		left.acceleration = 0.0;
		right.acceleration = 0.0;
	}

	public void updateSpeed(double lSpeed, double rSpeed, double time){
		left.updateSpeed(lSpeed, time);
		right.updateSpeed(rSpeed, time);
	}

	public void updateHeading (double heading){
		center.heading = heading;
		left.position = Geometry.inverseCenterLeft(center, WHEEL_BASE);
		right.position = Geometry.inverseCenterRight(center, WHEEL_BASE);
	}

	public void updateVoltage(double lVoltage, double rVoltage, double time){
		left.updateVoltage(lVoltage, time);
		right.updateVoltage(rVoltage, time);
	}
	
	public void updatePosition(double time) {
		double radius = radiusICC(WHEEL_BASE, left.velocity, right.velocity);
		double omega = velocityICC(WHEEL_BASE, left.velocity, right.velocity);
		double theta = omega * (time);
		double sinTheta = Math.sin(theta);
		double alpha = ((Math.PI) - theta)/2.0;
		double sinAlpha = Math.sin(alpha);

		double movementAngle = center.heading + theta;
		double leftMovement = Geometry.sideFromLawOfSines(radius+(WHEEL_BASE/2), sinAlpha, sinTheta);
		double rightMovement = Geometry.sideFromLawOfSines(radius-(WHEEL_BASE/2), sinAlpha, sinTheta);
		
		if (omega == 0.0) {
			leftMovement = -left.velocity * time;
			rightMovement = -right.velocity * time;
		}
		double sine = Math.sin(movementAngle);
		double cosine = Math.cos(movementAngle);
		double leftMovementX = -leftMovement*sine;
		double leftMovementY = -leftMovement*cosine;
		left.position.update(leftMovementX, leftMovementY);
		double rightMovementX = -rightMovement*sine;
		double rightMovementY = -rightMovement*cosine;
		right.position.update(rightMovementX, rightMovementY);
		center = Geometry.center(left.position, right.position);
		left.position = Geometry.inverseCenterLeft(center, WHEEL_BASE);
		right.position = Geometry.inverseCenterRight(center, WHEEL_BASE);
		
		//// Debug statements
		// System.out.println(center.x + ", " + center.y + ", " + center.heading);
//		 System.out.println("LV: " + left.velocity + "RV: " + right.velocity);
		// System.out.println(left.velocity + ", " + right.velocity);
		// System.out.println("LP: " + left.position.x + ", " + left.position.y);
		// System.out.println("RP: " + right.position.x + ", " + right.position.y);
		// System.out.println("MA: " + movementAngle + "Rad ICC: " + radius);
		// System.out.println("RM: " + rightMovement + " LM: " + leftMovement);
		// System.out.println(leftMovementX + "====" + leftMovementY);
		// System.out.println(rightMovementX + "====" + rightMovementY);
	}

	public void monitor(){

	}
	

	private static class DrivetrainSide{
		Pose position;
		double velocity;
		double acceleration;
		private double psuedoMass;
		private Boolean coast; 
		private CIMMotor cim = new CIMMotor();
				
		public DrivetrainSide(Pose position, double mass) {
			this.position = position;
			velocity = 0.0;
			acceleration = 0.0;
			psuedoMass = mass;
			coast = false;
		}

		public void updateSpeed(double speed, double time){
			double deltaVelocity = this.velocity - speed;
			this.acceleration = deltaVelocity/time;
			this.velocity = speed;
		}
		
		public void updateVoltage(double voltage, double time) {
			double motorSpeed = this.wheelSpeedToMotorSpeed(this.velocity);
			// double newAcceleration = this.wheelAcceleration(voltage, motorSpeed);
			
			double totalTorque = CIMMotor.outputTorque(voltage, motorSpeed) * GEAR_RATIO * CIMS_PER_SIDE;

			if (coast && Utils.threshold(voltage, 0.0, 0.05)) totalTorque = 0.0;
			double wheelForce = (totalTorque / WHEEL_RADIUS);
			
			double wheelnetForce = frictionModel(wheelForce, this.velocity);
			
			// Fake Traction Limiting
			double maxForce = 250.0;
			if (wheelnetForce < -maxForce) wheelnetForce = -maxForce;
			else if (wheelnetForce > maxForce) wheelnetForce = maxForce;
			
			double newAcceleration = wheelnetForce / psuedoMass;
			this.velocity += (newAcceleration + this.acceleration) / 2 * time; // Trapezoidal integration
			this.acceleration = newAcceleration;
			
		}

		protected double limitAcceleration(double outputVoltage){
			double motorSpeed = this.wheelSpeedToMotorSpeed(this.velocity);
			double maxVoltage = CIMMotor.inverseVoltage(MAX_TORQUE, motorSpeed)/CIMS_PER_SIDE/GEAR_RATIO;
			double minVoltage = CIMMotor.inverseVoltage(-MAX_TORQUE, motorSpeed)/CIMS_PER_SIDE/GEAR_RATIO;

			double limitedVoltage;
			if (outputVoltage > maxVoltage) limitedVoltage = maxVoltage;
			else if (outputVoltage < minVoltage) limitedVoltage = minVoltage;
			else limitedVoltage = outputVoltage;
			
			return outputVoltage;
		}
		
		/**
		 * Converts linear wheel speed back to motor angular speed
		 * @param speed meters/sec
		 * @return angular speed, revolutions per minute
		 */
		private double wheelSpeedToMotorSpeed(double speed) {
			double wheelCircum = WHEEL_RADIUS * 2 * Math.PI;
			double wheelRevs = speed / wheelCircum * 60.0;
			double motorRevs = wheelRevs * GEAR_RATIO;
			return motorRevs;
		}
		
		private double frictionModel(double force, double speed) {
			double netForce;
			if (speed == 0.0) {
				netForce = force;
			} else if (speed < 0.0) {
				netForce = force + DRIVETRAIN_FRICTION;
			} else if (speed > 0.0) {
				netForce = force - DRIVETRAIN_FRICTION;
			} else {
				netForce = 0.0;
			}
			return netForce;
		}
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
	private static double radiusICC(double wheelBase, double left, double right) {
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
	private static double velocityICC(double wheelBase, double left, double right) {
		return (right-left)/wheelBase;
	}
	
	// For Testing
	public static void main ( String[] args) {
		DrivetrainModel model = new DrivetrainModel();
		for (int i = 0; i < 300; i++) {
			model.updateVoltage(12.0, 12.0, 10.0/1000.0);
			model.updatePosition(10.0/1000.0);
		}
	}
}
