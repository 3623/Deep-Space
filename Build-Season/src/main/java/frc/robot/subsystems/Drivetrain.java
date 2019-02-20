package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.controls.PathFollower;
import frc.controls.WaypointNavigator;
import frc.util.Pose;
import frc.util.Tuple;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain {
	Spark rightMotors = new Spark(0);
	Spark leftMotors = new Spark(1);
	DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

	Encoder encLeft = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
	Encoder encRight = new Encoder(2, 3, true, Encoder.EncodingType.k4X);

	public DrivetrainModel model;
	private final double DISTANCE_PER_PULSE = model.WHEEL_RADIUS*Math.PI*2/2048.0;

	public WaypointNavigator waypointNav = new WaypointNavigator();

	double time;

	public Drivetrain(){
		model = new DrivetrainModel();

		encLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
		encRight.setDistancePerPulse(DISTANCE_PER_PULSE);
		model.setPosition(0.0, 0.0, 0.0);
	}

	public void writeToLog() {
    };

//    public void outputToSmartDashboard {	}

    public void stop() {
    	leftMotors.disable();
		rightMotors.disable();
	
	}

	public void updatemodel(double time) {
		model.updateSpeed(encLeft.getRate(), encRight.getRate(), time);
		model.updatePosition(time);

	}

   	public void zeroSensors() {
   		encLeft.reset();
	   	encRight.reset();	
	}
    
   public void openLoopControl(double xSpeed, double rSpeed, Boolean quickTurn) {
	   drivetrain.curvatureDrive(xSpeed, rSpeed, quickTurn);
   }

   public void directMotorControl(double leftSpeed, double rightSpeed){
	   drivetrain.tankDrive(leftSpeed, rightSpeed);
   }

   public void driveToWaypoint(){
		Pose goal = waypointNav.updatePursuit(model.center);
		Tuple out = PathFollower.driveToPoint(goal, model.center);
		double leftSpeed = out.left/4.0;
		double rightSpeed = out.right/4.0;
		directMotorControl(leftSpeed, rightSpeed);
   }

	public void update(double time){
		double deltaTime = time - this.time;
		this.time = time;
		this.updatemodel(deltaTime);
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