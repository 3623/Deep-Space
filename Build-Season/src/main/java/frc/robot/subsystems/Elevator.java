package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.modeling.motors.A775Pro;


public class Elevator{
    private Spark elevatorMotors;

    private Encoder elevatorEncoder;
    private static final double DISTANCE_PER_PULSE = Math.PI*1.125*2.0/2024.0;
    private final double OFFSET = 19.0;
    private DigitalInput bottomLimit = new DigitalInput(6);
    private final double BOTTOM_SOFT_LIMIT = 19.5;
    private DigitalInput topLimit = new DigitalInput(7);
    private final double TOP_SOFT_LIMIT = 78.0;

    private double goal;
    private final double MAX_GOAL = 77.75
    ;
    private final double MIN_GOAL = 19.5;

    private final double kP = 0.2/60.0;
    private final double kD = 0.0/60.0;
    private final double weightCompensation = 0.2/12.0;
    private final double deadband = 3.0;
    private Boolean isStopped;

    private double error;
    private double errorD;
    private double output;
    private double checkedOutput;

    private A775Pro  a775Pro = new A775Pro();
    private static final double MAX_CURRENT = 20.0;
    private static final double MOTORS_PER_SIDE = 4.0;
    private static final double GEAR_RATIO = 20.8;


    private Boolean isInverted = true;

    public Elevator(){
        elevatorMotors  = new Spark(2);
        elevatorMotors.setInverted(isInverted);

        elevatorEncoder = new Encoder(4, 5, isInverted, Encoder.EncodingType.k1X);
        // elevatorEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        goal = 0.0;
    }

    public void update(){
        double output = outputPD();
        double limitedOutput = limitCurrent(12.0*output, elevatorSpeedToMotorSpeed(elevatorSpeed()))/12.0;
        if (!isStopped){
            elevatorMotors.set(output);
        }
        zeroEncoder();
        monitor();
    }

    public void calibrate(double speed){
        elevatorMotors.set(speed);
    }

    private double outputPD(){
        error = goal - elevatorPosition();
        errorD = elevatorSpeed();
        output = (error*kP) - (errorD*kD);
        checkedOutput = checkLimit(output) + weightCompensation;
        if (withinDeadband()) checkedOutput = weightCompensation;
        return checkedOutput;
    }

    /** 
     * Limits acceleration using the models velocity and information about motors
     * @param unchecked output voltage
     * @return checked voltage, limited to acceleration of MAX_TORQUE constant
     */
    protected static double limitCurrent(double outputVoltage, double motorSpeed){
        double maxVoltage = A775Pro.inverseCurrent(MAX_CURRENT, motorSpeed);
        double minVoltage = A775Pro.inverseCurrent(-MAX_CURRENT, motorSpeed);


        double limitedVoltage;
        if (outputVoltage > maxVoltage) limitedVoltage = maxVoltage;
        else if (outputVoltage < minVoltage) limitedVoltage = minVoltage;
        else limitedVoltage = outputVoltage;

        System.out.println("Max: " + maxVoltage + ", Min: " + minVoltage + ", Limited: " + limitedVoltage);

        
        return limitedVoltage;
    }

    public void setGoal(double goal){
        if (goal > MAX_GOAL){
            this.goal = MAX_GOAL;
        } else if (goal < MIN_GOAL){
            this.goal = MIN_GOAL;
        } else {
            this.goal = goal;
        }
    }

    public void stop(){
        isStopped = true;
        elevatorMotors.stopMotor();
    }

    public void enable(){
        isStopped = false;
    }

    private void zeroEncoder(){
        if (!bottomLimit.get()){
            elevatorEncoder.reset();
        }
    }

    private Boolean withinDeadband(){
        if (Math.abs(elevatorPosition()-this.goal) < deadband){
            return true;
        } else {
            return false;
        }
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
        return (!bottomLimit.get() || elevatorPosition() < BOTTOM_SOFT_LIMIT);
    }

    private Boolean atTopLimit(){
        return (!topLimit.get() || elevatorPosition() > TOP_SOFT_LIMIT);
    }

    public double elevatorPosition(){
        return elevatorEncoder.getDistance()*DISTANCE_PER_PULSE+OFFSET;
    }

    private double elevatorSpeed(){
        return elevatorEncoder.getRate()*DISTANCE_PER_PULSE;
    }

    private static double elevatorSpeedToMotorSpeed(double elevatorSpeed){
        return elevatorSpeed/(Math.PI*1.125*2.0)*60.0*GEAR_RATIO;

    }

    private void monitor(){
        SmartDashboard.putNumber("Goal", goal);
        SmartDashboard.putNumber("Position", elevatorPosition());
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("P Val", error*kP);
        SmartDashboard.putNumber("D Val", errorD*kD);
        SmartDashboard.putNumber("Checked Output", checkedOutput);
        SmartDashboard.putBoolean("At Bottom", atBottomLimit());
        SmartDashboard.putBoolean("At Top", atTopLimit());
        System.out.println(DISTANCE_PER_PULSE);
    }

    // acceleration = (motorTorque/radius - gravity)/mass


    public static void main ( String[] args) {
        new A775Pro();
        for (int i = 0; i < 100; i++){
            double motorSpeed = Elevator.elevatorSpeedToMotorSpeed(i);
            double limitCurrent = Elevator.limitCurrent(12.0, motorSpeed);
            System.out.println(limitCurrent + " " + motorSpeed);
        }
    }
}