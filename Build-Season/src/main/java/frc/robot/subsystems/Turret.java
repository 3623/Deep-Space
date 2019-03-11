
package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Turret {
    Spark turretMotor;
    // zeroing sensor ??
    AnalogPotentiometer pot;

    private double goal;
    private double MAX_GOAL = 100.0;
    private double MIN_GOAL = -1000.0;

    private double kP = 1.0/180.0;
    private double kD = 0.2/180.0;

    private Boolean isStopped;

    private double error;
    private double errorD;
    private double output;

    private Pixy pixy;

    public Turret(){
        turretMotor = new Spark(6);
        pot = new AnalogPotentiometer(0, 180, -90);
        goal = 0.0;
    }

    public void update(){
        if (!isStopped){
            turretMotor.set(outputPD());
        }
        // monitor();
    }

    private double outputPD(){
        error = goal - pot.get();
        // errorD = pot.getRate();
        output = (error*kP) + (errorD*kD);
        return output;
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
        turretMotor.stopMotor();
    }

    public void enable(){
        isStopped = false;
    }

//     public void vision(){
//         double x = pixy.getTargetX();   
//     }

    public void manualControl(double speed){
        turretMotor.set(speed);
    }


    // public void turnToAngle(double desiredAngle, double currentAngle){
    //     double error = desiredAngle - currentAngle;
    //     double p = error / 180.0;
    //     turretMotor.set(p);

    // }
}
