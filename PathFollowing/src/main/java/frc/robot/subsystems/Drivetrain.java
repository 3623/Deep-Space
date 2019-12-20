package frc.robot.subsystems;

import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controls.CubicSplineFollower;
import frc.robot.commands.drive.DriverControl;
import frc.util.Tuple;
import frc.util.Utils;

public class Drivetrain extends Subsystem {
	private Spark rightMotors, leftMotors;

	private Encoder encLeft, encRight;

	private AHRS navx;

	private final int UPDATE_RATE = 200;
	public DrivetrainModel model;
	private final double DISTANCE_PER_PULSE = model.WHEEL_RADIUS * Math.PI * 2 / 2048.0;

	public CubicSplineFollower waypointNav;

	double time;

	private DriveControlState controlState = DriveControlState.DISABLED;

	private enum DriveControlState {
		OPEN_LOOP, // open loop voltage control
		PATH_FOLLOWING, // velocity PID control
		DISABLED,
	}

	public Drivetrain() {
		rightMotors = new Spark(0);
		leftMotors = new Spark(1);

		encLeft = new Encoder(0, 1, true, Encoder.EncodingType.k2X);
		encRight = new Encoder(2, 3, true, Encoder.EncodingType.k2X);
		encLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
		encRight.setDistancePerPulse(DISTANCE_PER_PULSE);

		navx = new AHRS(SPI.Port.kMXP);

		model = new DrivetrainModel();
		model.setPosition(0.0, 0.0, 0.0);

		waypointNav = new CubicSplineFollower();

		this.updateThreadStart();
	}

	@Override
	public void initDefaultCommand() {
		// setDefaultCommand(new DriverControl());
	}

	public void disable() {
		if (controlState != DriveControlState.DISABLED) {
			controlState = DriveControlState.DISABLED;
		}
		setMotorControllers(new Tuple(0.0, 0.0));
	}

	private void updateThreadStart() {
		Thread t = new Thread(() -> {
			while (!Thread.interrupted()) {
				this.update();
				try {
					Thread.sleep(1000 / UPDATE_RATE);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
		t.start();
	}

	private void updatePosition(double time) {
		model.updateSpeed(encLeft.getRate(), encRight.getRate(), time);
		model.updateHeading(navx.getAngle());
		model.updatePosition(time);
	}

	public void startPathFollowing() {
		controlState = DriveControlState.PATH_FOLLOWING;
	}

	private void driveWaypointNavigator() {
		Tuple output = waypointNav.updatePursuit(model.center);
		Tuple limitedOut = model.limitAcceleration(output);
		double leftSpeed = limitedOut.left;
		double rightSpeed = limitedOut.right;
		SmartDashboard.putNumber("Left Out", leftSpeed);
		SmartDashboard.putNumber("Right Out", rightSpeed);
		// setVoltages(leftSpeed, rightSpeed);
	}

	public void zeroSensors() {
		encLeft.reset();
		encRight.reset();
		navx.reset();
	}

	public void driverControl(double xSpeed, double rSpeed, Boolean quickTurn) {
		setOpenLoop(new Tuple(0.0, 0.0));
		// setOpenLoop(xSpeed, rSpeed);
	}

	private void update() {
		double time = Timer.getFPGATimestamp();
		double deltaTime = time - this.time;
		this.time = time;
		this.updatePosition(deltaTime);
		this.monitor();
		SmartDashboard.putNumber("DT", deltaTime);

		switch (controlState) {
		case OPEN_LOOP:
			break;
		case PATH_FOLLOWING:
			driveWaypointNavigator();
			break;
		case DISABLED:
			break;
		}
	}

	public void setOpenLoop(Tuple out) {
		if (controlState != DriveControlState.OPEN_LOOP) {
			System.out.println("Switching to open loop control, time: " + time);
			controlState = DriveControlState.OPEN_LOOP;
		}
		setMotorControllers(out);
	}

	private void setMotorControllers(double left, double right) {
		leftMotors.set(left);
		rightMotors.set(right);
	}

	private void setMotorControllers(Tuple out) {
		setMotorControllers(out.left, out.right);
	}

	private void setVoltages(double left, double right) {
		setMotorControllers(left / 12.0, right / 12.0);
	}

	private void setVoltages(Tuple voltages) {
		setVoltages(voltages.left, voltages.right);
	}

	private void monitor() {
		// SmartDashboard.putNumber("Left Encoder", encLeft.getDistance());
		// SmartDashboard.putNumber("Rights Encoder", encRight.getDistance());
		SmartDashboard.putNumber("Drivetrain Model X", model.center.x);
		SmartDashboard.putNumber("Drivetrain Model Y", model.center.y);
		SmartDashboard.putNumber("Heading", model.center.heading);
		// SmartDashboard.putNumber("Drivetrain Heading", navx.getAngle());
	}

	public static void main(String[] args) throws IOException {
	}

}