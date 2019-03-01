/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.controls.Waypoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  DriverStation DS;  
  
  Drivetrain drivetrain;	 
  Grabber grabber;
  Elevator elevator;

	Joystick driverController, steeringWheel;
  XboxController operatorController;
  
  Timer timer;

  SendableChooser<String> autoChooser;
  String autoSelected;
  final String CrossLine = "Cross Line";
  final String DriveBy = "Drive Forward and Plop";

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    DS = DriverStation.getInstance();
    timer = new Timer();

    drivetrain = new Drivetrain();	 
    grabber = new Grabber();
    elevator = new Elevator();
  
    driverController = new Joystick(0);
    steeringWheel = new Joystick(1);
    operatorController = new XboxController(2);

    autoChooser = new SendableChooser<String>();
    autoChooser.addDefault(CrossLine, CrossLine);
    autoChooser.addObject(DriveBy, DriveBy);
    SmartDashboard.putData("Auto choices", autoChooser);

    drivetrain.zeroSensors();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
	  drivetrain.update(timer.getFPGATimestamp());
    elevator.update(); 
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoSelected = autoChooser.getSelected();

    switch(autoSelected){
      case CrossLine:
        drivetrain.model.setPosition(4.2, 1.3, 0.0);
        drivetrain.waypointNav.addWaypoint(new Waypoint(4.2, 3.3, 0.0, 0.3, 0.5));
      break;

      case DriveBy:
        drivetrain.model.setPosition(4.2, 1.3, 0.0);
        drivetrain.waypointNav.addWaypoint(new Waypoint(4.2, 1.3, 0.0));
        drivetrain.waypointNav.addWaypoint(new Waypoint(5.5, 2.5, 0.0, 0.4, 0.7));
        drivetrain.waypointNav.addWaypoint(new Waypoint(5.5, 3.5, 0.0, 0.3, 0.4));
      break;
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
      drivetrain.driveToWaypoint();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // Drivetrain
    if (driverController.getTrigger()){
      drivetrain.openLoopControl(-driverController.getRawAxis(1)/2.0, 
      driverController.getRawAxis(3)/2.0, 
      driverController.getTrigger());
    } else{
      drivetrain.openLoopControl(-driverController.getRawAxis(1), 
      driverController.getRawAxis(3)*Math.abs(driverController.getRawAxis(3)), 
      driverController.getTrigger());
    }
    
    // Grabber
    if (operatorController.getAButton()) grabber.intake();
    else if (operatorController.getBButton()) grabber.place();
    else grabber.defaultState();

    // Elevator
    if (operatorController.getPOV() == 180) elevator.setGoal(0.0);
    else if (operatorController.getPOV() == 90) elevator.setGoal(28.0);
    else if (operatorController.getPOV() == 0) elevator.setGoal(56.0);

    // Turret

  }

  @Override
  public void testInit() {
    elevator.enable();
    elevator.setGoal(30.0);
  }


  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  @Override
  public void disabledInit() {
    elevator.stop();
  }
}
