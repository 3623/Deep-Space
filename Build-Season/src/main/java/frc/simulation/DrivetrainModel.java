package frc.simulation;

import frc.simulation.motors.CIMMotor;
import frc.simulation.motors.Motor;
import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Utils;
import frc.simulation.Kinematics;

/**
 * Model for a differential drivetrain, simulates mass and motor performance
 * Integrates acceleration over time for both sides of the drivetrain and uses velocity to 
 * calculate the instantaneous curvature of the robot, which is then integrated for position
 * @author eric
 *
 */
public class DrivetrainModel {

	private final double DRIVETRAIN_MASS = 63.5; // kg
	private final double WHEEL_BASE = 0.762; // meters
	private final double CENTER_MASS = 0.381; // from left wheel
	private Boolean COAST_MODE = false; 
	public Pose center;
	private DrivetrainSide left, right;
	
	public DrivetrainModel(double x, double y, double heading) {
		center = new Pose(x, y, heading); // Initial robot position
		left = new DrivetrainSide(Geometry.inverseCenterLeft(center, WHEEL_BASE),
									DRIVETRAIN_MASS/2);
		right = new DrivetrainSide(Geometry.inverseCenterRight(center, WHEEL_BASE),
									DRIVETRAIN_MASS/2);	
		
		if (COAST_MODE) {
			left.setCoast();
			right.setCoast();
		} else {
			left.setBrake();
			right.setBrake();
		}
	}
	
//	public DrivetrainModel() {
//		center = new Pose(0.0, 0.0, 0.0); // Initial robot position
//		left = new DrivetrainSide(Geometry.inverseCenterLeft(center, WHEEL_BASE),
//				DRIVETRAIN_MASS/2);
//		right = new DrivetrainSide(Geometry.inverseCenterRight(center, WHEEL_BASE),
//				DRIVETRAIN_MASS/2);	
//	}

	public void updateSpeed(double lSpeed, double rSpeed, double time){
		left.velocity = lSpeed;
		right.velocity = rSpeed;

	}

	public void updateVoltage(double lVoltage, double rVoltage, double time){
		left.updateVoltage(lVoltage, time);
		right.updateVoltage(rVoltage, time);
	}
	
	public void updatePosition(double time) {

		double radius = Kinematics.radiusICC(WHEEL_BASE, left.velocity, right.velocity);
		double omega = Kinematics.velocityICC(WHEEL_BASE, left.velocity, right.velocity);
		double theta = omega * (time);
		double sinTheta = Math.sin(theta);
		double alpha = (Math.PI/4.0) - (theta/2.0);
		double sinAlpha = Math.sin(alpha);

		double movementAngle = center.heading - (theta/2.0);
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
		
		// Debug statements
//		System.out.println(center.x + ", " + center.y + ", " + center.heading);
//		System.out.println("LV: " + left.velocity + "RV: " + right.velocity);
		System.out.println(left.velocity + ", " + right.velocity);
//		System.out.println("LP: " + left.position.x + ", " + left.position.y);
//		System.out.println("RP: " + right.position.x + ", " + right.position.y);
//		System.out.println("MA: " + movementAngle + "Rad ICC: " + radius);
//		System.out.println("RM: " + rightMovement + " LM: " + leftMovement);
//		System.out.println(leftMovementX + "====" + leftMovementY);
//		System.out.println(rightMovementX + "====" + rightMovementY);
	}
	
	/**
	 * 
	 * @author eric
	 *
	 */
	private static class DrivetrainSide{
		Pose position;
		double velocity;
		private double acceleration;
		private double psuedoMass;
		private Boolean coast; 
		
		private static final double WHEEL_RADIUS = 0.0508; // meters
		private static final double CIMS_PER_SIDE = 2.0; // Minicim is 0.58
		private static final double GEAR_RATIO = 10.7/1.0; // Reduction
		// Future can make motors a list of motors rather than coefficient
//		private static final CIMMotor cimL1, cimL2, cimR1, cimR2 = new CIMMotor();
//		private static final Motor[] motorsL = {cimL1, cimL2};
		private static final double FRICTION = 115;
		
		static CIMMotor cim = new CIMMotor();
		
		public DrivetrainSide(Pose position, double mass) {
			this.position = position;
			velocity = 0.0;
			acceleration = 0.0;
			psuedoMass = mass;
			coast = false;
		}

		public void updateSpeed(double speed, double time){
			double deltaVelocity = speed - this.velocity;
			this.acceleration = deltaVelocity/time;
			this.velocity = speed;
		}
		
		public void updateVoltage(double voltage, double time) {
			double motorSpeed = this.wheelSpeedToMotorSpeed(this.velocity);
//			double newAcceleration = this.wheelAcceleration(voltage, motorSpeed);
			
			double totalTorque = CIMMotor.outputTorque(voltage, motorSpeed) * GEAR_RATIO * CIMS_PER_SIDE;
			if (coast && Utils.threshold(voltage, 0.0, 0.05)) totalTorque = 0.0;
			double wheelForce = (totalTorque / WHEEL_RADIUS);
			

			double wheelnetForce = frictionModel(wheelForce, this.velocity);
			
			if (wheelnetForce < -150.0) wheelnetForce = -150.0;
			else if (wheelnetForce > 150.0) wheelnetForce = 150.0;
			
//			System.out.println(wheelForce + " " + wheelnetForce);
			double newAcceleration = wheelnetForce / psuedoMass;
			
			this.velocity += (newAcceleration + this.acceleration) / 2 * time; // Trapezoidal integration
			this.acceleration = newAcceleration;

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
				netForce = force + FRICTION;
			} else if (speed > 0.0) {
				netForce = force - FRICTION;
			} else {
				netForce = 0.0;
			}
			return netForce;
		}

		
		public void setBrake() {
			if (coast) coast = false;
		}
		
		public void setCoast() {
			if (!coast) coast = true;
		}
	}
	
	// Testing
	public static void main ( String[] args) {
		DrivetrainModel model = new DrivetrainModel(0.0, 0.0, 0.0);
		for (int i = 0; i < 300; i++) {
			model.updateVoltage(12.0, 12.0, 10.0/1000.0);
			model.updatePosition(10.0/1000.0);
		}
	}
}
