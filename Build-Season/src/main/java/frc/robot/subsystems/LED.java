package frc.robot.subsystems;

import com.mach.LightDrive.LightDriveCAN;
import java.awt.Color;

public class LED{
    LightDriveCAN lightDrive = new LightDriveCAN();

    public void test(){
        lightDrive.SetColor(1, Color.WHITE, 1.0);
    }
}