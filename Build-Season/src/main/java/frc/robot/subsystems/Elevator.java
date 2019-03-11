package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.modeling.motors.A775Pro;

public class Elevator extends PIDSubsystem {
	private Spark elevatorMotor1, elevatorMotor2, elevatorMotor3, elevatorMotor4;
    private SpeedControllerGroup elevatorMotors;

    private Encoder elevatorEncoder;
    private static final double DISTANCE_PER_PULSE = Math.PI*1.125*2.0/2024.0;
    private final double OFFSET = 19.0;
    private DigitalInput bottomLimit = new DigitalInput(6);
    private final double BOTTOM_SOFT_LIMIT = 19.5;
    private DigitalInput topLimit = new DigitalInput(7);
    private final double TOP_SOFT_LIMIT = 75.0;

    private double goal;
    private final double MAX_GOAL = 70.75
    ;
    private final double MIN_GOAL = 19.5;

    private final static double kP = 0.15/60.0;
    private final static double kD = 0.0/60.0;
    private final double weightCompensation = 0.3/12.0;
    private final double DEADBAND = 3.0;

    private double output;
    private double checkedOutput;
    private double limitedOutput;

    private A775Pro  a775Pro = new A775Pro();
    private static final double MAX_CURRENT = 15.0;
    private static final double MOTORS_PER_SIDE = 4.0;
    private static final double GEAR_RATIO = 20.8;


    private Boolean isInverted = true;
	

	public Elevator() {
		super("Lift", kP, kD, 0.0);
		setInputRange(MIN_GOAL, MAX_GOAL);
		setOutputRange(-0.2, 0.2);
        setAbsoluteTolerance(DEADBAND);

        elevatorMotor1  = new Spark(2);
        elevatorMotor2  = new Spark(3);
        elevatorMotor3  = new Spark(4);
        elevatorMotor4  = new Spark(5);
        elevatorMotors = new SpeedControllerGroup(elevatorMotor1, elevatorMotor2, elevatorMotor3, elevatorMotor4);
        elevatorMotors.setInverted(isInverted);

        elevatorEncoder = new Encoder(4, 5, isInverted, Encoder.EncodingType.k1X);
        elevatorEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

	}

	public void initDefaultCommand() {
	}
    
	protected double returnPIDInput() {
		return elevatorEncoder.getDistance() + OFFSET; // returns the sensor value that is providing the feedback for the system
	}

	protected void usePIDOutput(double output) {
        this.output = output;
        checkedOutput = checkLimit(output) + weightCompensation;
        double motorSpeed = elevatorSpeedToMotorSpeed(elevatorEncoder.getRate());
        limitedOutput = limitCurrent(12.0*checkedOutput, motorSpeed)/12.0;

        elevatorMotors.set(checkedOutput); // this is where the computed output value from the PIDController is applied to the motor
        SmartDashboard.putNumber("Checked Output", checkedOutput);
        SmartDashboard.putNumber("Motor Speed", motorSpeed);
        SmartDashboard.putNumber("Limited Output", limitedOutput);
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
        return (!bottomLimit.get() || elevatorEncoder.getDistance() < BOTTOM_SOFT_LIMIT);
    }

    private Boolean atTopLimit(){
        return (!topLimit.get() || elevatorEncoder.getDistance() > TOP_SOFT_LIMIT);
    }

    private void zeroEncoder(){
        if (!bottomLimit.get()){
            elevatorEncoder.reset();
        }
    }

    public double elevatorPosition(){
        return elevatorEncoder.getDistance();
    }

    public void updateStuff(){
        zeroEncoder();
        monitor();
    }

    private static double elevatorSpeedToMotorSpeed(double elevatorSpeed){
        return elevatorSpeed/(Math.PI*1.125*2.0)*60.0*GEAR_RATIO;
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

//         System.out.println("Max: " + maxVoltage + ", Min: " + minVoltage + ", Limited: " + limitedVoltage);

        
        return limitedVoltage;
    }

    private void monitor(){
        SmartDashboard.putNumber("Goal", this.getSetpoint());
        SmartDashboard.putNumber("Position", this.getPosition());
        SmartDashboard.putNumber("Speed", elevatorEncoder.getRate());
 	SmartDashboard.putBoolean("At Bottom", atBottomLimit());
        SmartDashboard.putBoolean("At Top", atTopLimit());
    }


}
