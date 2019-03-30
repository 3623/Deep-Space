package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;

import java.io.IOException;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.modeling.motors.A775Pro;

public class Elevator extends PIDSubsystem {
    private Spark elevatorMotors;
    private Boolean isInverted = true;

    private Encoder elevatorEncoder;
    private static final double DISTANCE_PER_PULSE = Math.PI*1.125*2.0/2024.0;
    private final double OFFSET = 20.0;
    private DigitalInput bottomLimit = new DigitalInput(6);
    private final double BOTTOM_SOFT_LIMIT = 25.5;
    private DigitalInput topLimit = new DigitalInput(7);
    private final double TOP_SOFT_LIMIT = 77.0;

    private final double MAX_GOAL = 77.0;
    private final double MIN_GOAL = 20.0;

    private final static double kP = 1.0/60.0;
    private final static double kI = 0.0/60.0;
    private final static double kD = 0.0/60.0;
    private final double weightCompensation = 0.16;
    private final double DEADBAND = 1.0;

    private static final double MAX_CURRENT = 15.0;
    private static final double MOTORS = 4.0;
    private static final double GEAR_RATIO = 20.8;
    private static final double SPOOL_RADIUS = 0.0142875;
    private static final double MASS = 13.0;
    private static final double MAX_ACCELERATION = 10.0;
    private static final double MAX_FORCE_POSITIVE = (MAX_ACCELERATION+10.0)*MASS;
    private static final double MAX_FORCE_NEGATIVE = (-MAX_ACCELERATION+10.0)*MASS;
    private static final double MAX_TORQUE_POSITIVE = MAX_FORCE_POSITIVE*SPOOL_RADIUS;
    private static final double MAX_TORQUE_NEGATIVE = MAX_FORCE_NEGATIVE*SPOOL_RADIUS;
	

	public Elevator() {
        super("Lift", kP, kI, kD);
		setInputRange(MIN_GOAL, MAX_GOAL);
		setOutputRange(-0.6, 0.6);
        setAbsoluteTolerance(DEADBAND);

        elevatorMotors = new Spark(2);
        elevatorMotors.setInverted(isInverted);

        elevatorEncoder = new Encoder(4, 5, isInverted, Encoder.EncodingType.k1X);
        elevatorEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        new A775Pro();
	}

     /** 
     * Limits acceleration using the models velocity and information about motors
     * @param unchecked output voltage
     * @return checked voltage, limited to acceleration of MAX_TORQUE constant
     */
    protected static double limitCurrent(double outputVoltage, double motorSpeed){
        double maxVoltage = A775Pro.currentToVoltage(MAX_CURRENT, motorSpeed);
        double minVoltage = A775Pro.currentToVoltage(-MAX_CURRENT, motorSpeed);

        double limitedVoltage;
        if (outputVoltage > maxVoltage) limitedVoltage = maxVoltage;
        else if (outputVoltage < minVoltage) limitedVoltage = minVoltage;
        else limitedVoltage = outputVoltage;

        // System.out.println("Max: " + maxVoltage + ", Min: " + minVoltage + ", Limited: " + limitedVoltage);

        return limitedVoltage;
    }

    protected static double limitAcceleration(double outputVoltage, double motorSpeed){
        double maxVoltage = A775Pro.torqueToVoltage(MAX_TORQUE_POSITIVE/MOTORS/GEAR_RATIO, motorSpeed);
        double minVoltage = A775Pro.torqueToVoltage(MAX_TORQUE_NEGATIVE/MOTORS/GEAR_RATIO, motorSpeed);

        double limitedVoltage;
        if (outputVoltage > maxVoltage) limitedVoltage = maxVoltage;
        else if (outputVoltage < minVoltage) limitedVoltage = minVoltage;
        else limitedVoltage = outputVoltage;

        // System.out.println("Max: " + maxVoltage + ", Min: " + minVoltage + ", Limited: " + limitedVoltage);

        return limitedVoltage;
    }
    
	protected double returnPIDInput() {
		return elevatorEncoder.getDistance() + OFFSET; // returns the sensor value that is providing the feedback for the system
	}

	protected void usePIDOutput(double output) {
        double checkedOutput = checkLimit(output);
        double motorSpeed = elevatorSpeedToMotorSpeed(elevatorEncoder.getRate());
        double limitedOutput = limitAcceleration(12.0*checkedOutput, motorSpeed)/12.0;
       
        double finalOutput;
        if (onTarget() && atBottomLimit()){
            finalOutput = 0.0;
        } else if (onTarget()){
            finalOutput = weightCompensation; 
        } else if (this.getSetpoint() > this.getPosition()){
            finalOutput = checkedOutput+weightCompensation; 
        } else if (this.getSetpoint() < this.getPosition()){
            finalOutput = checkedOutput;
        } else {
            finalOutput = 0.0;
        }
        elevatorMotors.set(finalOutput); 

        SmartDashboard.putNumber("Elevator Checked Output", checkedOutput);
        SmartDashboard.putNumber("Elevator Final Output", finalOutput);
        SmartDashboard.putNumber("Elevator Motor Speed", motorSpeed);
        SmartDashboard.putNumber("Elevator Limited Output", limitedOutput);
	}

    public void update(){
        zeroEncoder();
        monitor();
    }

    private void monitor(){
        SmartDashboard.putNumber("Elevator Goal", this.getSetpoint());
        SmartDashboard.putNumber("Elevator Position", this.getPosition());
        SmartDashboard.putNumber("Elevator Speed", elevatorEncoder.getRate());
 	    SmartDashboard.putBoolean("Elevator At Bottom", atBottomLimit());
        SmartDashboard.putBoolean("Elevator At Top", atTopLimit());
    }

    private void zeroEncoder(){
        if (!bottomLimit.get()){
            elevatorEncoder.reset();
        }
    }

    private static double elevatorSpeedToMotorSpeed(double elevatorSpeed){
        return elevatorSpeed/(Math.PI*1.125*2.0)*60.0*GEAR_RATIO;
    }

   

    private double checkLimit(double motorOutput){
        double limitedOutput;
        if (atTopLimit() && motorOutput > 0.0){
            limitedOutput = 0.0;
        } else if (atBottomLimit() && motorOutput < 0.0){
            limitedOutput = 0.0;
        } else {
            limitedOutput = motorOutput;
        }
        return limitedOutput;
    }

    private Boolean atBottomLimit(){
        return (!bottomLimit.get() || this.getPosition() < BOTTOM_SOFT_LIMIT);
    }

    private Boolean atTopLimit(){
        return (!topLimit.get() || this.getPosition() > TOP_SOFT_LIMIT);
    }

    public void initDefaultCommand() {}

    public void calibrate(double val){
        double checkedOutput = checkLimit(val);
        SmartDashboard.putNumber("Elevator Checked Output", checkedOutput);
        elevatorMotors.set(checkedOutput);
    }
    
    public static void main ( String[] args ) throws IOException {
        new A775Pro();
        double speed = 0.0;
        limitCurrent(12.0, speed);
        limitAcceleration(12.0, 10000);
    }

}
