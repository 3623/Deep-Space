package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.controls.PathFollower;
import frc.controls.WaypointNavigator;
import frc.simulation.DrivetrainModel;
import frc.util.Pose;
import frc.util.Tuple;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveTrain {
	Spark rightMotors = new Spark(0);
	Spark leftMotors = new Spark(1);
	DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

	Encoder encLeft = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
	Encoder encRight = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
	public WaypointNavigator waypointNav = new WaypointNavigator();


	public DrivetrainModel position;
	double distancePerPulse = 0.486/2048.0;


	double time;

	public DriveTrain(){
		position = new DrivetrainModel();

		encLeft.setDistancePerPulse(distancePerPulse);
		encRight.setDistancePerPulse(distancePerPulse);
		position.setPosition(0.0, 0.0, 0.0);
	}

	public void writeToLog() {
    };

//    public void outputToSmartDashboard {	}

    public void stop() {
    	leftMotors.disable();
		rightMotors.disable();
	
	}

	public void updatePosition(double time) {
		position.updateSpeed(encLeft.getRate(), encRight.getRate(), time);
		position.updatePosition(time);

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

   public void driveToPoint(){
		Pose goal = waypointNav.updatePursuit(position.center);
		Tuple out = PathFollower.driveToPoint(goal, position.center);
		double leftSpeed = out.left/4.0;
		double rightSpeed = out.right/4.0;
		directMotorControl(leftSpeed, rightSpeed);
   }

//    public void registerEnabledLoops(Looper enabledLooper) {
//	}

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
		SmartDashboard.putNumber("Position X", position.center.x);
		SmartDashboard.putNumber("Position Y", position.center.y);
		SmartDashboard.putNumber("Heading", position.center.heading*180/Math.PI);

	}
    
    public static void main(String[] args) throws IOException{}
    







}