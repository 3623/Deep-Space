
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
    private static final double SCALE_FACTOR = 340.0;
    private static final double OFFSET = 25.0;

    private double MAX_GOAL = 275.0;
    private double MIN_GOAL = 85.0;
    private static final double TOP_SOFT_LIMIT = 260.0;
    private static final double BOTTOM_SOFT_LIMIT = 80.0;

    private static final double kP = 6.0/180.0;
    private static final double kI = 0.001/180.0;
    private static final double kD = 0.2/180.0;
    private static final double DEADBAND = 1.0;


    private Pixy pixy;

    public Turret(){
        super(kP, kI, kD);
        setInputRange(MIN_GOAL, MAX_GOAL);
		setOutputRange(-1.0, 1.0);
        setAbsoluteTolerance(DEADBAND);
        
        turretMotor = new Spark(6);
        turretMotor.setInverted(true);

        pot = new AnalogPotentiometer(0, SCALE_FACTOR, OFFSET);

        pixy = new Pixy();
    }

    public void vision(){
        PixyPacket[] blocks = pixy.readBlocks();
        SmartDashboard.putNumber("Targets", blocks.length);
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

    public void updateStuff(){
        monitor();
        // vision();
    }

    public void monitor(){
        SmartDashboard.putNumber("Turret Angle", this.getPosition());
        SmartDashboard.putNumber("Turret Goal", this.getSetpoint());
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

    public void manualControl(double speed){
        turretMotor.set(speed);
    }

    @Override
    protected void initDefaultCommand() {
    }
}
