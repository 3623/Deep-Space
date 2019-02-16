package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.simulation.DrivetrainModel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveTrain {
	Spark rightMotors = new Spark(0);
	Spark leftMotors = new Spark(1);
	DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

	Encoder encLeft = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder encRight = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
	DrivetrainModel model = new DrivetrainModel();
	double distancePerPulse = 0.508/2048.0;

	double time;

	public DriveTrain(){
		encLeft.setDistancePerPulse(distancePerPulse);
		encRight.setDistancePerPulse(distancePerPulse);
		model.setPosition(0.0, 0.0, 0.0);
	}

	public void writeToLog() {
    };

//    public void outputToSmartDashboard {	}

    public void stop() {
    	leftMotors.disable();
		rightMotors.disable();
	
	}

	public void updatePosition(double time) {
		model.updateSpeed(encLeft.getRate(), encRight.getRate(), time);
		model.updatePosition(time);

	}

   	public void zeroSensors() {
   		encLeft.reset();
	   	encRight.reset();	
		model.setPosition(0.0, 0.0, 0.0);

	}
    
   public void openLoopControl(double xSpeed, double rSpeed) {
	   drivetrain.curvatureDrive(xSpeed, rSpeed, false);
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
		SmartDashboard.putNumber("Position X", model.center.x);
		SmartDashboard.putNumber("Position Y", model.center.y);
		SmartDashboard.putNumber("Heading", model.center.heading);

	}
    
    public static void main(String[] args) throws IOException{}
    







}