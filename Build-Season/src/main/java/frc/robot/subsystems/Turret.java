
package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.turret.TurretManualControl;
import frc.util.Utils;

public class Turret extends PIDSubsystem {
    Spark turretMotor;
    // zeroing sensor ??
    AnalogPotentiometer pot;
    private static final double SCALE_FACTOR = 340.0;
    private static final double OFFSET = 15.0;

    private double MAX_GOAL = 275.0;
    private double MIN_GOAL = 85.0;
    private static final double TOP_SOFT_LIMIT = 275.0;
    private static final double BOTTOM_SOFT_LIMIT = 85.0;

    private static final double kP = 1/20.0;
    private static final double kI = 0.001/180.0;
    private static final double kD = 0.2/180.0;
    private static final double DEADBAND = 1.0;

    private Pixy2 pixy;
    private List<PixyPacket> pixyBlocks;
    private static final double FRAME_X = 315;
    private static final double FRAME_Y = 217;
    private static final double FOV = 60.0;
    private static final double TARGET_Y = 170.0;
    private static final double EPSILON_Y = 40.0;

    private int targetCount = 0;
    private double visionOffset = 0.0;


    public Turret(){
        super(kP, kI, kD);
        setInputRange(MIN_GOAL, MAX_GOAL);
		setOutputRange(-1.0, 1.0);
        setAbsoluteTolerance(DEADBAND);
        
        turretMotor = new Spark(3);
        turretMotor.setInverted(false);

        pot = new AnalogPotentiometer(0, SCALE_FACTOR, OFFSET);

        pixy = new Pixy2();
    }

    public void vision() throws IOException {
        pixyBlocks = pixy.readBlocks();
        if(pixyBlocks == null){
            // SmartDashboard.putString("Targets", "Null");
        } else {
            double xTotal = 0.0;
            targetCount = 0;
            System.out.println(pixyBlocks.size());
            for(PixyPacket block : pixyBlocks){
                if(checkPixyBlock(block)){
                    targetCount++;
                    xTotal += block.xCenter;
                }
            }

            if (targetCount == 2) {
                double xCenter = xTotal/targetCount;
                double xOffset = xCenter-(FRAME_X/2.0);
                visionOffset = xOffset/FRAME_X*FOV;
                SmartDashboard.putNumber("Targets", targetCount);
                SmartDashboard.putNumber("X Center", visionOffset);
            }
        }
    }

    public Boolean checkPixyBlock(PixyPacket block){
        Boolean centerYGood = Utils.withinThreshold(block.yCenter, TARGET_Y, EPSILON_Y);
        return centerYGood;
    }

    public void setSpeed(double speed){
        this.setSetpoint(this.getPosition() + speed*20.0);
    }

    public void setVisionControlled(){
        if (targetCount == 2.0){
            this.setSetpoint(this.getPosition() + visionOffset);
        }
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

    public void update() {
        monitor();
        try {
            vision();
        } catch (IOException e) {
            e.printStackTrace();
        }
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
        setDefaultCommand(new TurretManualControl());
    }
}
