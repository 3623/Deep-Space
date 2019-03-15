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
import frc.robot.commands.auto.LeftRocket;
import frc.robot.commands.drive.DriveOffLevel1;
import frc.robot.commands.auto.LeftCargoShip;
import frc.robot.commands.grabber.Intake;
import frc.robot.commands.grabber.Place;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Turret;

import org.opencv.core.Point;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;


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

  Thread m_visionThread;


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
  
    driverController = new XboxController(0);
    operatorController = new XboxController(1);

    autoChooser = new SendableChooser<String>();
    autoChooser.setDefaultOption(CrossLine, CrossLine);
    autoChooser.addOption(LeftCargoShip, LeftCargoShip);
    autoChooser.addOption(LeftRocket, LeftRocket);
    autoChooser.addOption(DriverControl, DriverControl);
    SmartDashboard.putData("Auto choices", autoChooser);

    driverControl = false;

    m_visionThread = new Thread(() -> {
      // Get the Axis camera from CameraServer
      AxisCamera camera
          = CameraServer.getInstance().addAxisCamera("axis-camera.local");
      // Set the resolution
      camera.setResolution(320, 240);
      camera.setFPS(15);

      // Get a CvSink. This will capture Mats from the camera
      CvSink cvSink = CameraServer.getInstance().getVideo();
      // Setup a CvSource. This will send images back to the Dashboard
      CvSource outputStream
          = CameraServer.getInstance().putVideo("Rectangle", 320, 240);

      // Mats are very memory expensive. Lets reuse this Mat.
      Mat mat = new Mat();

      // This cannot be 'true'. The program will never exit if it is. This
      // lets the robot stop this thread when restarting robot code or
      // deploying.
      while (!Thread.interrupted()) {
        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat.  If there is an error notify the output.
        if (cvSink.grabFrame(mat) == 0) {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          // skip the rest of the current iteration
          continue;
        }
        outputStream.putFrame(mat);
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
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
        driverControl = true;
    }

    autoCommand.start();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
      // drivetrain.driveToWaypoint();
      
      if (driverControl) teleopInit();
      else{
        Scheduler.getInstance().run();
      }

  }

  @Override
  public void teleopInit() {
    elevator.enable();
    elevator.setSetpoint(elevator.elevatorPosition());
    turret.enable();
    Scheduler.getInstance().removeAll();
  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    // Drivetrain
	  Boolean quickTurn;
    if (Math.abs(driverController.getRawAxis(1)) < 0.1) quickTurn = true;
    else quickTurn = false;
    quickTurn = true;

    if (driverController.getRawButton(9)){
      drivetrain.openLoopControl(-driverController.getRawAxis(1)/2.0, 
      driverController.getRawAxis(4)/2.0,
      quickTurn);
    } else{
      drivetrain.openLoopControl(-driverController.getRawAxis(1)*Math.abs(driverController.getRawAxis(1)), 
      driverController.getRawAxis(4)/2.0, 
      quickTurn);
    }
    
    // Grabber
    if (operatorController.getBButtonPressed()) Scheduler.getInstance().add(new Place());
    else if (operatorController.getAButtonPressed()) Scheduler.getInstance().add(new Intake());

    // Elevator
    if (operatorController.getPOV() == 180) elevator.setSetpoint(19.0);
    else if (operatorController.getPOV() == 90) elevator.setSetpoint(49.0);
    else if (operatorController.getPOV() == 0) elevator.setSetpoint(75.0);

    // Turret
    // Manual Control w/o Potentiometer
    turret.manualControl(operatorController.getRawAxis(4));

    
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
    turret.setSetpoint(220.0);
  }

  @Override
  public void disabledInit() {
    elevator.disable();
    turret.disable();
  }
}
