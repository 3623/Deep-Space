package frc.modeling;

import javax.imageio.ImageIO;
import javax.swing.*;

import frc.controls.CubicSplineFollower;
import frc.controls.DrivetrainControls;
import frc.controls.CubicSplineFollower.Waypoint;
import frc.controls.DrivetrainControls;
import frc.robot.subsystems.DrivetrainModel;
import frc.util.Pose;
import frc.util.Tuple;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class Animation extends JPanel implements Runnable
{
	protected Thread sim;      // animation thread
	protected int width;          // width of viewing area in pixels
	protected int height;         // height of viewing area in pixels
	protected Dimension size;     // size of viewing area
	protected Image image;        // off-screen image
	protected Graphics offScreen; // off-screen graphics
	protected Image field, robot;
	protected final double speed = 4.0;
	protected final double scale = 65; // pixels per meter
	protected static final int offsetX = 5;
	protected static final int offsetY = 15;
	private static final int SPEED = 1;
	protected final int x;
	protected final int y;
	protected int robotWidth, robotHeight;
	protected ArrayList<Tuple> trajectory;

	protected int dt = 20;          // interval between frames in millisec
	private DrivetrainModel model;
	private CubicSplineFollower nav;

	protected double time = 0.0;

	public Animation() throws IOException {
		field = ImageIO.read(new File("field-blue.png"));
		robot = ImageIO.read(new File("robot-blue.png"));

		// Set the width and heigth and size
		width = field.getWidth(this);
		height = field.getHeight(this);
		robotWidth = robot.getWidth(this);
		robotHeight = robot.getHeight(this);
		setSize(width, height);	
		x = 0 + offsetX;	
		y = height-offsetY;
		
		model = new DrivetrainModel();
		nav = new CubicSplineFollower();

		// model.setPosition(5.0, 5.0, 0.0);
		// nav.addWaypoint(new Waypoint(6.5, 6.0, 0.0, 0.7));
		// nav.addWaypoint(new Waypoint(4.5, 6.0, 0.0, 0.7));

		model.setPosition(3.0, 3.0, 0.0);
		nav.addWaypoint(new Waypoint(3.3, 4.0, 35.0, 0.7));
		nav.addWaypoint(new Waypoint(5.0, 5.7, 40.0, 0.7));
		nav.addWaypoint(new Waypoint(5.5, 7.5, 0.0, 0.7));
	
		sim = new Thread ( this );	// Create and start the thread
		sim.start();

		trajectory = new ArrayList<Tuple>();
	}


	// Update function
	public void paintComponent (Graphics g) {  
		double simTime = dt*1.0/1000.0/speed;
		time += simTime;
		
		Tuple output = nav.updatePursuit(model.center);
		double leftVoltage = output.left*12.0;
		double rightVoltage = output.right*12.0;

		//System.out.println("Left Voltage: " + leftVoltage + ", Right Voltage: " + rightVoltage);

		model.updateVoltage(leftVoltage, rightVoltage, simTime);
		model.updatePosition(simTime);

		size = this.getSize(); // Get the size of the viewing area
		if ( image == null ) { 	// Create the off-screen image buffer if it is the first time
			image = createImage ( size.width, size.height);
			offScreen = image.getGraphics();
		}

		offScreen.drawImage(field, 0, 0, this);	// Draw background field

		/// Draw robot
		int xCoord = x + (int) Math.round(model.center.x * scale)-(robotWidth/2);
		int yCoord = y - (int) Math.round(model.center.y * scale)-(robotHeight/2);
		trajectory.add(new Tuple(xCoord + (robotWidth/2), yCoord + (robotHeight/2)));
		AffineTransform tx = AffineTransform.getRotateInstance(model.center.r, robotWidth/2, robotHeight/2);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		offScreen.drawImage(op.filter((BufferedImage) robot, null), xCoord, yCoord, this);

		xCoord = x + (int) Math.round(nav.getCurrentWaypoint().x * scale);
		yCoord = y - (int) Math.round(nav.getCurrentWaypoint().y * scale);
		offScreen.setColor(Color.yellow);
		offScreen.drawOval(xCoord, yCoord, 6, 6);
		offScreen.setColor(Color.blue);
		offScreen.drawOval(xCoord, yCoord, 1, 1);

		for(Tuple point : trajectory){
			offScreen.setColor ( Color.yellow );
			offScreen.drawOval((int) point.left, (int) point.right, 1, 1);
		}

		/// Copy the off-screen image to the screen
		g.drawImage ( image, 0, 0, this );     

	}

	@Override
	public void update ( Graphics g ) {
		paintComponent ( g );
	}

	@Override
	public void run () {
		while ( Thread.currentThread() == sim  && nav.getIsFinished() == false) {
			repaint ();
			try {
				Thread.sleep ( dt*SPEED );
			} catch (InterruptedException e) {
				System.out.println ("Exception: " + e.getMessage());
			}        
		}
	}

	public static void main ( String[] args ) throws IOException {
		JFrame frame = new JFrame ( "Drivetrain Simulation" );
		Animation panel = new Animation ();
		frame.getContentPane().add ( panel );
		frame.setSize ( panel.width, panel.height  + 35);
		frame.setVisible ( true );
		frame.addWindowListener (new WindowAdapter() {
			public void windowClosing ( WindowEvent evt ) {
				System.exit ( 0 );
			}
		});
	}
}