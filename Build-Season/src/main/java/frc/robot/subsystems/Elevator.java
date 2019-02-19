package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator{
    private Spark bottomMotors = new Spark(2);
    private Spark topMotors = new Spark(3);
    private  SpeedControllerGroup elevatorMotors = new SpeedControllerGroup(bottomMotors, topMotors);

    private Encoder elevatorEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
    private double distancePerPulse = (Math.PI*1*2)/2048.0;
    private DigitalInput bottomLimit = new DigitalInput(6);
    private double bottomSoftLimit = 0.5;
    private DigitalInput topLimit = new DigitalInput(7);
    private double topSoftLimit = 58.0;

    private double goal;
    private double maxGoal = 56.0;
    private double minGoal = 0.0;

    private double kP = 1.0/56.0;
    private double kD = 0.2/56.0;
    private double weightCompensation = 0.06;

    private Boolean isStopped;

    private double error;
    private double errorD;
    private double output;
    private double checkedOutput;

    public Elevator(){
        elevatorEncoder.setDistancePerPulse(distancePerPulse);
        goal = 0.0;
    }

    public void update(){
        if (!isStopped){
            elevatorMotors.set(outputPD());
        }
        zeroEncoder();
        monitor();
    }

    private double outputPD(){
        error = goal - elevatorEncoder.getDistance();
        errorD = elevatorEncoder.getRate();
        output = (error*kP) + (errorD*kD);
        checkedOutput = checkLimit(output) + weightCompensation;
        return checkedOutput;
    }

    public void setGoal(double goal){
        if (goal > maxGoal){
            this.goal = maxGoal;
        }
        else if (goal < minGoal){
            this.goal = minGoal;
        }
        else this.goal = goal;
    }

    public void stop(){
        isStopped = true;
        elevatorMotors.stopMotor();
    }

    public void enable(){
        isStopped = false;
    }

    private void zeroEncoder(){
        if (bottomLimit.get()){
            elevatorEncoder.reset();
        }
    }

    private double checkLimit(double motorOutput){
        double limitedOutput;
        if (atTopLimit() && motorOutput > 0.0){
            limitedOutput = 0.0;
        }
        else if (atBottomLimit() && motorOutput < 0.0){
            limitedOutput = 0.0;
        }
        else limitedOutput = motorOutput;
        return limitedOutput;
    }

    private Boolean atBottomLimit(){
        return (bottomLimit.get() || elevatorEncoder.getDistance() < bottomSoftLimit);
    }

    private Boolean atTopLimit(){
        return (topLimit.get() || elevatorEncoder.getDistance() > topSoftLimit);
    }

    private void monitor(){
        SmartDashboard.putNumber("Goal", goal);
        SmartDashboard.putNumber("Position", elevatorEncoder.getDistance());
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("P Val", error*kP);
        SmartDashboard.putNumber("D Val", errorD*kD);
        SmartDashboard.putNumber("Checked Output", checkedOutput);
        SmartDashboard.putBoolean("At Bottom", atBottomLimit());
        SmartDashboard.putBoolean("At Top", atTopLimit());
    }
}