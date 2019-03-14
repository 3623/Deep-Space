
package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Turret extends PIDSubsystem{
    Spark turretMotor;
    // zeroing sensor ??
    AnalogPotentiometer pot;

    private double goal;
    private double MAX_GOAL = 280.0;
    private double MIN_GOAL = 80.0;

    private static final double kP = 1.0/180.0;
    private static final double kI = 1.0/180.0;
    private static final double kD = 0.2/180.0;
    private static final double DEADBAND = 5;

    private Boolean isStopped;

    private Pixy pixy;

    public Turret(){
        super(kP, kI, kD);
        setInputRange(MIN_GOAL, MAX_GOAL);
		setOutputRange(-0.8, 0.8);
        setAbsoluteTolerance(DEADBAND);
        
        turretMotor = new Spark(6);
        pot = new AnalogPotentiometer(0, 180, -90);

        pixy = new Pixy();
    }

    public void update(){
        
        // monitor();
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

//     public void vision(){
//         double x = pixy.getTargetX();   
//     }

    public void manualControl(double speed){
        turretMotor.set(speed);
    }

    @Override
    protected double returnPIDInput() {
        return 0;
    }

    @Override
    protected void usePIDOutput(double output) {

    }

    @Override
    protected void initDefaultCommand() {
    }


    // public void turnToAngle(double desiredAngle, double currentAngle){
    //     double error = desiredAngle - currentAngle;
    //     double p = error / 180.0;
    //     turretMotor.set(p);

    // }
}
