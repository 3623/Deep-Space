
package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends PIDSubsystem{
    Spark turretMotor;
    // zeroing sensor ??
    AnalogPotentiometer pot;

    private double goal;
    private double MAX_GOAL = 260.0;
    private double MIN_GOAL = 100.0;

    private static final double kP = 2.0/180.0;
    private static final double kI = 0.0/180.0;
    private static final double kD = 0.2/180.0;
    private static final double DEADBAND = 5;
    private static final double TOP_SOFT_LIMIT = 260.0;
    private static final double BOTTOM_SOFT_LIMIT = 100.0;

    private Pixy pixy;

    public Turret(){
        super(kP, kI, kD);
        setInputRange(MIN_GOAL, MAX_GOAL);
		setOutputRange(-0.5, 0.5);
        setAbsoluteTolerance(DEADBAND);
        
        turretMotor = new Spark(6);
        turretMotor.setInverted(true);
        pot = new AnalogPotentiometer(0, 540, -315);

        pixy = new Pixy();
    }

    public void updateStuff(){
        monitor();
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
        return (this.getPosition() < BOTTOM_SOFT_LIMIT);
    }

    private Boolean atTopLimit(){
        return (this.getPosition() > TOP_SOFT_LIMIT);
    }

    public void vision(){
		PixyPacket[] blocks = pixy.readBlocks();
    }

    public void manualControl(double speed){
        turretMotor.set(speed);
    }

    @Override
    protected double returnPIDInput() {
        return pot.get();
    }

    @Override
    protected void usePIDOutput(double output) {
        double checkedOutput = checkLimit(output);

        turretMotor.set(checkedOutput);
        SmartDashboard.putNumber("Turret Output", checkedOutput);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void monitor(){
        SmartDashboard.putNumber("Turret Angle", this.getPosition());
        SmartDashboard.putNumber("Turret Goal", this.getSetpoint());
    }
}
