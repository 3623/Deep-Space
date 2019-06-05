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

public class Animation extends JPanel implements Runnable {
	protected Thread sim; // animation thread
	protected int width; // width of viewing area in pixels
	protected int height; // height of viewing area in pixels
	protected Dimension size; // size of viewing area
	protected Image image; // off-screen image
	protected Graphics offScreen; // off-screen graphics
	protected Image field, robot;
	protected final double scale = 65; // pixels per meter
	protected final int x;
	protected final int y;
	protected int robotWidth, robotHeight;
	protected ArrayList<Tuple> trajectory;// array for storing points passed through by robot

	protected final int SPEED = 1; // replay speed
	protected final int FRAME_RATE = 60; // interval between frames in millisec

	private DrivetrainModel model;
	private CubicSplineFollower nav;

	protected final int ODOMETRY_UPDATE_RATE = 200;
	protected final int CONTROL_UPDATE_RATE = 200;
	protected double leftVoltage = 0.0;
	protected double rightVoltage = 0.0;


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
		x = 0 + width / 2; // the x offset for drawing objects
		y = height - 15; // y offset for drawing objects

		model = new DrivetrainModel();
		nav = new CubicSplineFollower();

		// model.setPosition(0.0, 1.0, 0.0);
		// nav.addWaypoint(new Waypoint(0.3, 4.0, 40.0, 1.0));
		// // nav.addWaypoint(new Waypoint(2.0, 5.7, 220.0, -0.7));
		// nav.addWaypoint(new Waypoint(2.5, 7.5, 0.0, 1.0));

		// model.setPosition(0.0, 1.0, 0.0);
		// nav.addWaypoint(new Waypoint(0.3, 2.0, -10.0, 1.0));

		// model.setPosition(1.2, 0.7, 0.0);
		// nav.addWaypoint(new Waypoint(3.3, 5.0, 30.0, 1.0));
		// nav.addWaypoint(new Waypoint(3.4, 0.5, 0.0, -1.0));

		// Side of cargo ship x2
		model.setPosition(1.2, 0.7, 0.0);
		nav.addWaypoint(new Waypoint(1.2, 3.0, 0.0, 0.5, false));
		nav.addWaypoint(new Waypoint(1.1, 6.6, -10.0, 1.0, true));
		nav.addWaypoint(new Waypoint(3.5, 0.5, 0.0, -1.0, true));
		nav.addWaypoint(new Waypoint(1.1, 7.2, -10.0, 1.0, true));

		// model.setPosition(-1.2, 0.7, 0.0);
		// nav.addWaypoint(new Waypoint(-1.2, 3.0, 0.0, 0.5, false));
		// nav.addWaypoint(new Waypoint(-1.2, 6.6, 0.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(-3.5, 0.5, 0.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(-1.2, 7.2, 0.0, 1.0, true));

		// // Right rocket
		// model.setPosition(1.2, 0.7, 0.0);
		// nav.addWaypoint(new Waypoint(1.2, 3.0, 0.0, 0.5));
		// nav.addWaypoint(new Waypoint(2.9, 6.3, 60.0, 1.0));
		// nav.addWaypoint(new Waypoint(3.5, 6.6, 60.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(2.9, 6.3, 60.0, -1.0));
		// nav.addWaypoint(new Waypoint(3.5, 0.5, 0.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(3.7, 3.5, 0.0, 1.0));
		// nav.addWaypoint(new Waypoint(3.4, 5.0, -60.0, 1.0, true));

		// model.setPosition(-1.2, 0.7, 0.0);
		// nav.addWaypoint(new Waypoint(-1.2, 3.0, 0.0, 0.5, false));
		// nav.addWaypoint(new Waypoint(-2.9, 6.3, -60.0, 1.0));
		// nav.addWaypoint(new Waypoint(-3.5, 6.6, -60.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(-2.9, 6.3, -60.0, -1.0));
		// nav.addWaypoint(new Waypoint(-3.5, 0.5, -0.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(-3.7, 3.5, -0.0, 1.0));
		// nav.addWaypoint(new Waypoint(-3.4, 5.0, 60.0, 1.0, true));


		// model.setPosition(0.0, 1.0, 0.0);
		// // nav.addWaypoint(new Waypoint(0.4, 3.0, 90.0, 1.0));
		// // nav.addWaypoint(new Waypoint(2.0, 3.4, 0.0, 1.0));
		// nav.addWaypoint(new Waypoint(1.0, 3.0, 90.0, 1.0, true));

		// nav.addWaypoint(new Waypoint(3.3, 5.0, 0.0, 1.0));

		sim = new Thread(this); // Create and start the thread
		sim.start();
		odometryThread();
		controlThread();

		trajectory = new ArrayList<Tuple>();
	}

	// Update function
	public void paintComponent(Graphics g) {
		size = this.getSize(); // Get the size of the viewing area
		if (image == null) { // Create the off-screen image buffer if it is the first time
			image = createImage(size.width, size.height);
			offScreen = image.getGraphics();
		}

		offScreen.drawImage(field, 0, 0, this); // Draw background field

		/// Draw robot
		int xCoord = x + (int) Math.round(model.center.x * scale) - (robotWidth / 2);
		int yCoord = y - (int) Math.round(model.center.y * scale) - (robotHeight / 2);
		if(!nav.getIsFinished()) trajectory.add(new Tuple(xCoord + (robotWidth / 2), yCoord + (robotHeight / 2)));
		AffineTransform tx = AffineTransform.getRotateInstance(model.center.r, robotWidth / 2, robotHeight / 2);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		offScreen.drawImage(op.filter((BufferedImage) robot, null), xCoord, yCoord, this);

		// Draw waypoints
		xCoord = x + (int) Math.round(nav.getCurrentWaypoint().x * scale);
		yCoord = y - (int) Math.round(nav.getCurrentWaypoint().y * scale);
		offScreen.setColor(Color.yellow);
		offScreen.drawOval(xCoord - 3, yCoord - 3, 6, 6);

		// Draw trajectory
		for (Tuple point : trajectory) {
			offScreen.setColor(Color.yellow);
			offScreen.drawOval((int) point.left, (int) point.right, 1, 1);
		}

		/// Copy the off-screen image to the screen
		g.drawImage(image, 0, 0, this);
	}

	@Override
	public void update(Graphics g) {
		paintComponent(g);
	}

	@Override
	public void run() {
		while (Thread.currentThread() == sim && nav.getIsFinished() == false) {
			repaint();
			try {
				Thread.sleep(1000/FRAME_RATE);
			} catch (InterruptedException e) {
				System.out.println("Exception: " + e.getMessage());
			}
		}
	}

	public void controlThread(){
		Thread c = new Thread(() -> {
            while (!Thread.interrupted()) {

				Tuple output = nav.updatePursuit(model.center);
				output = model.limitAcceleration(output);
				leftVoltage = output.left * 12.0;
				rightVoltage = output.right * 12.0;

				System.out.println("Left Voltage: " + leftVoltage + ", Right Voltage: " +
				rightVoltage);

				if(nav.getIsFinished()){
				}

				try {
					Thread.sleep(1000/CONTROL_UPDATE_RATE * SPEED);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
        });
        c.start();
	}

	public void odometryThread(){
		Thread o = new Thread(() -> {
            while (!Thread.interrupted()) {
				time += 1000/ODOMETRY_UPDATE_RATE;

				// leftVoltage += (Math.random() - 0.5) * 4.0; // Some random error. Well, a lot
				// rightVoltage += (Math.random() - 0.5) * 4.0;

				model.updateVoltage(leftVoltage, rightVoltage, 1.0/ODOMETRY_UPDATE_RATE);
				model.updatePosition(1.0/ODOMETRY_UPDATE_RATE);
				System.out.println(model.center.x);

				try {
					Thread.sleep(1000/ODOMETRY_UPDATE_RATE * SPEED);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
        });
        o.start();
	}

	public static void main(String[] args) throws IOException {
		JFrame frame = new JFrame("Drivetrain Simulation");
		Animation panel = new Animation();
		frame.getContentPane().add(panel);
		frame.setSize(panel.width, panel.height + 35);
		frame.setVisible(true);
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent evt) {
				System.exit(0);
			}
		});
	}
}