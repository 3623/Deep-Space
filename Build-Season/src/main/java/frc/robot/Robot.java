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

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.commands.auto.LeftRocket;
import frc.robot.commands.drive.DriveOffLevel1;
import frc.robot.commands.drive.ManualControl;
import frc.robot.commands.GeneralTimer;
import frc.robot.commands.auto.LeftCargoShip;
import frc.robot.commands.grabber.Hold;
import frc.robot.commands.grabber.Intake;
import frc.robot.commands.grabber.Place;
import frc.robot.subsystems.AxisCameraStream;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Turret;
import frc.util.Utils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  DriverStation DS;  
  
  public static Drivetrain drivetrain;	 
  public static Grabber grabber;
  public static Elevator elevator;
  public static Turret turret;

  AxisCameraStream axisCam;

  public static XboxController driverController, operatorController;
  
  Timer timer;

  SendableChooser<String> autoChooser;
  String autoSelected;
  final String CrossLine = "Cross Line";
  final String LeftCargoShip = "Left Cargo Ship";
  final String LeftRocket = "Left Rocket Ship";
  final String DriverControl = "Driver Control";

  Boolean driverControl;

  Command autoCommand;



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
    turret = new Turret();

    axisCam = new AxisCameraStream();
  
    driverController = new XboxController(0);
    operatorController = new XboxController(1);

    autoChooser = new SendableChooser<String>();
    autoChooser.setDefaultOption(DriverControl, DriverControl);
    autoChooser.addOption(LeftCargoShip, LeftCargoShip);
    autoChooser.addOption(LeftRocket, LeftRocket);
    autoChooser.addOption(CrossLine, CrossLine);
    SmartDashboard.putData("Auto choices", autoChooser);

    driverControl = false;
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
    elevator.updateStuff(); 
    turret.updateStuff();
    
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
    drivetrain.zeroSensors();
    autoSelected = autoChooser.getSelected();
    Scheduler.getInstance().removeAll();

    turret.enable();

    switch(autoSelected){
      case CrossLine:
        autoCommand = new DriveOffLevel1();
      break;

      case LeftCargoShip:
        autoCommand = new LeftCargoShip();
      break;

      case LeftRocket:
        autoCommand = new LeftRocket();
      break;

      case DriverControl:
        autoCommand = new GeneralTimer(0.0);
        driverControl = true;
      break;
    }

    autoCommand.start();

    if (driverControl) teleopInit();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
      // drivetrain.driveToWaypoint();
      
      if (driverControl) teleopPeriodic();
      else if (!operatorController.getRawButton(5)){
        Scheduler.getInstance().run();
      }

  }

  @Override
  public void teleopInit() {
    elevator.enable();
    turret.enable();
    Scheduler.getInstance().removeAll();

    elevator.setSetpoint(elevator.getPosition());
    turret.setSetpoint(180);
  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    // Drivetrain is now a default command
    
    // Grabber
    if (operatorController.getBButtonPressed()) Scheduler.getInstance().add(new Place());
    else if (operatorController.getAButtonPressed()) Scheduler.getInstance().add(new Intake());

    // Elevator
    if (operatorController.getPOV() == 180) elevator.setSetpoint(19.0);
    else if (operatorController.getPOV() == 90) elevator.setSetpoint(49.0);
    else if (operatorController.getPOV() == 0) elevator.setSetpoint(75.0);

    // Turret
    if (Math.abs(operatorController.getRawAxis(0)) > 0.1){
        // Manual control with pot
      turret.setSetpoint(turret.getPosition() + operatorController.getRawAxis(0)*17.0);
      System.out.println("SHAT");
    } else if (Utils.outsideDeadband(operatorController.getRawAxis(4), 0.0, 0.3) ||
    Utils.outsideDeadband(operatorController.getRawAxis(5), 0.0, 0.3)){
        // Setpoint control
       double goalAngle = (Math.toDegrees(Math.atan2(operatorController.getRawAxis(4), -operatorController.getRawAxis(5)))+360.0)%360.0;
       double robotAngle = drivetrain.model.center.heading;
      turret.setSetpoint(((goalAngle - robotAngle)+360.0)%360.0);
    }
    // Manual Control w/o Potentiometer
    // turret.manualControl(operatorController.getRawAxis(0)/2.0);
    
  }

  @Override
  public void testInit() {
    turret.enable();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    turret.setSetpoint(180.0);
  }

  @Override
  public void disabledInit() {
    elevator.disable();
    turret.disable();
    drivetrain.stop();
  }
}
