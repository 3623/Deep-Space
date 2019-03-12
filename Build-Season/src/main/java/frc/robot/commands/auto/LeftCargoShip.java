/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drive.DriveOffLevel1;
import frc.robot.commands.drive.LeftCargoShipToLeftLoadingZone;
import frc.robot.commands.drive.LeftHabToLeftCargoShip;
import frc.robot.commands.drive.LeftLoadingZoneToLeftCargoShip2;

public class LeftCargoShip extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftCargoShip() {
    addSequential(new DriveOffLevel1());
    addSequential(new LeftHabToLeftCargoShip());
    addSequential(new LeftCargoShipToLeftLoadingZone());
    addSequential(new LeftLoadingZoneToLeftCargoShip2());
    addSequential(new LeftCargoShipToLeftLoadingZone());
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
