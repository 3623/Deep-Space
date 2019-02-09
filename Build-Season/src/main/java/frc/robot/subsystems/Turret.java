
package frc.robot.subsystems;
import java.io.IOException;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Turret {
    Spark turretMotor = new Spark(4);
    // zeroing sensor ??
    AnalogInput pot = new AnalogInput(0);
    double angle = 0;

    public void stop() {
        turretMotor.disable();
    }

    public void turnToAngle(double desiredAngle, double currentAngle){
        double error = desiredAngle - currentAngle;
        double p = error / 180.0;
        turretMotor.set(p);

    }
    
    private void updateAngle(){
        angle = pot.getVoltage();
        
    }

    public void update(){
        updateAngle();

    }
}