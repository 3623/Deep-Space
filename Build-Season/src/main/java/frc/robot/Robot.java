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
	
  Drivetrain drivetrain = new Drivetrain();	 
  Grabber grabber = new Grabber();
  Elevator elevator = new Elevator();

	Joystick driverController = new Joystick(0);
  Joystick steeringWheel = new Joystick(1);
  XboxController operatorController = new XboxController(2);
  
  Timer timer = new Timer();



  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // SmartDashboard.putData("Auto choices", );
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
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);

      drivetrain.model.setPosition(4.2, 1.3, 0.0);
			drivetrain.waypointNav.addWaypoint(new Waypoint(4.2, 1.3, 0.0));
			drivetrain.waypointNav.addWaypoint(new Waypoint(5.5, 2.5, 0.0, 0.4, 0.7));
			drivetrain.waypointNav.addWaypoint(new Waypoint(5.5, 3.5, 0.0, 0.3, 0.4));

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
      drivetrain.driveToWaypoint();
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // Drivetrain
    drivetrain.openLoopControl(-steeringWheel.getRawAxis(1), 
      steeringWheel.getRawAxis(3)*Math.abs(steeringWheel.getRawAxis(3)), 
      steeringWheel.getTrigger());
    
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
