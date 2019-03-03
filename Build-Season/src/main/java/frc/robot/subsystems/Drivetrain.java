package frc.robot.subsystems;

import java.io.IOException;
// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.controls.PathFollower;
import frc.controls.WaypointNavigator;
import frc.util.Pose;
import frc.util.Tuple;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain {
	Spark rightMotors, leftMotors;
	DifferentialDrive drivetrain;

	Encoder encLeft, encRight;

	// AHRS navx; 

	public DrivetrainModel model;
	private final double DISTANCE_PER_PULSE = model.WHEEL_RADIUS*Math.PI*2/2048.0;

	public WaypointNavigator waypointNav;

	double time;

	public Drivetrain(){
		rightMotors = new Spark(0);
		leftMotors = new Spark(1);
		drivetrain = new DifferentialDrive(leftMotors, rightMotors);

		encLeft = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
		encRight = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
		encLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
		encRight.setDistancePerPulse(DISTANCE_PER_PULSE);

		// navx = new AHRS(SerialPort.Port.kMXP);

		model = new DrivetrainModel();
		model.setPosition(0.0, 0.0, 0.0);

		waypointNav = new WaypointNavigator();
	}

    public void stop() {
    	leftMotors.disable();
		rightMotors.disable();
	
	}

	public void updatePosition(double time) {
		model.updateSpeed(encLeft.getRate(), encRight.getRate(), time);
		model.updatePosition(time);
	}

	public void driveToWaypoint(){
		Pose goal = waypointNav.updatePursuit(model.center);
		Tuple out = PathFollower.driveToPoint(goal, model.center);
		double leftSpeed = out.left*0.8;
		double rightSpeed = out.right*0.8;
		directMotorControl(leftSpeed, rightSpeed);
   }

   	public void zeroSensors() {
   		encLeft.reset();
		encRight.reset();
		// navx.reset();   	
	}
    
   	public void openLoopControl(double xSpeed, double rSpeed, Boolean quickTurn) {
	   	drivetrain.curvatureDrive(xSpeed, rSpeed, quickTurn);
   	}

   	public void directMotorControl(double leftSpeed, double rightSpeed){
	  	drivetrain.tankDrive(leftSpeed, rightSpeed, false);
   	}

	public void update(double time){
		double deltaTime = time - this.time;
		this.time = time;
		this.updatePosition(deltaTime);
		this.monitor();
		SmartDashboard.putNumber("DT", deltaTime);
	}
 
	public void monitor(){
		SmartDashboard.putNumber("Left Encoder", encLeft.getDistance());
		SmartDashboard.putNumber("Rights Encoder", encRight.getDistance());
		SmartDashboard.putNumber("model X", model.center.x);
		SmartDashboard.putNumber("model Y", model.center.y);
		SmartDashboard.putNumber("Heading", model.center.heading*180/Math.PI);
	}
    
    public static void main(String[] args) throws IOException{}
    

}