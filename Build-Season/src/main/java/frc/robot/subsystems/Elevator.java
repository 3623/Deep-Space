package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;

import java.io.IOException;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.modeling.motors.A775Pro;

public class Elevator extends PIDSubsystem {
	private Spark elevatorMotors;

    private Encoder elevatorEncoder;
    private static final double DISTANCE_PER_PULSE = Math.PI*1.125*2.0/2024.0;
    private final double OFFSET = 20.0;
    private DigitalInput bottomLimit = new DigitalInput(6);
    private final double BOTTOM_SOFT_LIMIT = 20.5;
    private DigitalInput topLimit = new DigitalInput(7);
    private final double TOP_SOFT_LIMIT = 75.0;

    private final double MAX_GOAL = 70.75;
    private final double MIN_GOAL = 20.0;

    private final static double kP = 0.15/60.0;
    private final static double kI = 0.01/60.0;
    private final static double kD = 0.05/60.0;
    private final double weightCompensation = 0.4/12.0;
    private final double DEADBAND = 3.0;

    private double checkedOutput;
    private double limitedOutput;

    private A775Pro  a775Pro = new A775Pro();
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



    private Boolean isInverted = true;
	

	public Elevator() {
        super("Lift", kP, kI, kD);
		setInputRange(MIN_GOAL, MAX_GOAL);
		setOutputRange(-0.15, 0.15);
        setAbsoluteTolerance(DEADBAND);

        elevatorMotors = new Spark(2);
        elevatorMotors.setInverted(isInverted);

        elevatorEncoder = new Encoder(4, 5, isInverted, Encoder.EncodingType.k1X);
        elevatorEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
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

        System.out.println("Max: " + maxVoltage + ", Min: " + minVoltage + ", Limited: " + limitedVoltage);

        return limitedVoltage;
    }

    protected static double limitAcceleration(double outputVoltage, double motorSpeed){
        double maxVoltage = A775Pro.torqueToVoltage(MAX_TORQUE_POSITIVE/MOTORS/GEAR_RATIO, motorSpeed);
        double minVoltage = A775Pro.torqueToVoltage(MAX_TORQUE_NEGATIVE/MOTORS/GEAR_RATIO, motorSpeed);

        double limitedVoltage;
        if (outputVoltage > maxVoltage) limitedVoltage = maxVoltage;
        else if (outputVoltage < minVoltage) limitedVoltage = minVoltage;
        else limitedVoltage = outputVoltage;

        System.out.println("Max: " + maxVoltage + ", Min: " + minVoltage + ", Limited: " + limitedVoltage);

        return limitedVoltage;
    }
    
	protected double returnPIDInput() {
		return elevatorEncoder.getDistance() + OFFSET; // returns the sensor value that is providing the feedback for the system
	}

	protected void usePIDOutput(double output) {
        checkedOutput = checkLimit(output);
        double motorSpeed = elevatorSpeedToMotorSpeed(elevatorEncoder.getRate());
        limitedOutput = limitAcceleration(12.0*checkedOutput, motorSpeed)/12.0;

        SmartDashboard.putNumber("Checked Output", checkedOutput);
        SmartDashboard.putNumber("Motor Speed", motorSpeed);
        SmartDashboard.putNumber("Limited Output", limitedOutput);

        if (onTarget()){
            elevatorMotors.set(weightCompensation); // this is where the computed output value from the PIDController is applied to the motor
        } else{
            elevatorMotors.set(checkedOutput); // this is where the computed output value from the PIDController is applied to the motor
        }        
	}

    public void updateStuff(){
        zeroEncoder();
        monitor();
    }

    private void monitor(){
        SmartDashboard.putNumber("Goal", this.getSetpoint());
        SmartDashboard.putNumber("Position", this.getPosition());
        SmartDashboard.putNumber("Speed", elevatorEncoder.getRate());
 	    SmartDashboard.putBoolean("At Bottom", atBottomLimit());
        SmartDashboard.putBoolean("At Top", atTopLimit());
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
    
    public static void main ( String[] args ) throws IOException {
        new A775Pro();
        limitCurrent(12.0, 0.0);
        limitAcceleration(12.0, 0.0);
    }

}
