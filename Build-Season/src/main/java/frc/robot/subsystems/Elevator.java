package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator{
    private  Spark elevatorMotors;

    private Encoder elevatorEncoder;
    private final double DISTANCE_PER_PULSE = Math.PI*1.125*2.0/2024.0;
    private final double OFFSET = 19.0;
    private DigitalInput bottomLimit = new DigitalInput(6);
    private final double BOTTOM_SOFT_LIMIT = 19.5;
    private DigitalInput topLimit = new DigitalInput(7);
    private final double TOP_SOFT_LIMIT = 78.0;

    private double goal;
    private final double MAX_GOAL = 77.75
    ;
    private final double MIN_GOAL = 19.5;

    private final double kP = 0.9/60.0;
    private final double kD = 0.3/60.0;
    private final double weightCompensation = 0.06;

    private Boolean isStopped;

    private double error;
    private double errorD;
    private double output;
    private double checkedOutput;

    private Boolean isInverted = false;

    public Elevator(){
        elevatorMotors  = new Spark(2);
        elevatorMotors.setInverted(isInverted);

        elevatorEncoder = new Encoder(4, 5, isInverted, Encoder.EncodingType.k1X);
        // elevatorEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        goal = 0.0;
    }

    public void update(){
        double output = outputPD();
        if (!isStopped){
            elevatorMotors.set(output);
        }
        zeroEncoder();
        monitor();
    }

    private double outputPD(){
        error = goal - elevatorPosition();
        errorD = elevatorEncoder.getRate()*DISTANCE_PER_PULSE;
        output = (error*kP) + (errorD*kD);
        checkedOutput = checkLimit(output) + weightCompensation;
        return checkedOutput;
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

    private double elevatorPosition(){
        return elevatorEncoder.getDistance()*DISTANCE_PER_PULSE+OFFSET;
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
}