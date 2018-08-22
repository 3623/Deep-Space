package org.usfirst.frc.team3623.simulation;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

public class Animation extends JPanel implements Runnable
{
  protected Thread sim;      // animation thread
  protected int width;          // width of viewing area in pixels
  protected int height;         // height of viewing area in pixels
  protected int x, y;           // current position of text
  protected int dt;          // interval between frames in millisec
  protected Dimension size;     // size of viewing area
  protected Image image;        // off-screen image
  protected Graphics offScreen; // off-screen graphics
  private DrivetrainModel model;
  
  public Animation( int aWidth, int aHeight )
  {
    // Set the width and heigth and size
    width = aWidth;
    height = aHeight;
    setSize ( width, height );
  
    
    // Set the time interval between frames in millisec
    dt = 30;
    
    // Set the intial values for x and y
    x = width / 2;
    y = height / 2;
    
    model = new DrivetrainModel();
    

    // Create and start the thread
    sim = new Thread ( this );
    sim.start();
  }
  
  
  // Update function
  public void paintComponent ( Graphics g )
  {  
	  model.update(12.0, 11.0, dt/2/1000.0);

      // Get the size of the viewing area
      size = this.getSize();
      
      // Create the off-screen image buffer if it is the first time
      if ( image == null )
      {
          image = createImage ( size.width, size.height );
          offScreen = image.getGraphics();
      }
        
      // Set the color and paint the background
      offScreen.setColor ( Color.black );
      offScreen.fillRect ( 0, 0, size.width, size.height );
      
      // Set the pen color and draw the text
      int xCoord = (int) (model.center.x * 10) + x;
      int yCoord = y - (int) (model.center.y * 10);

      offScreen.setColor ( Color.yellow );
      offScreen.drawOval(xCoord, yCoord, 4, 4);
      
      // Copy the off-screen image to the screen
      g.drawImage ( image, 0, 0, this );     
  }
  
  @Override
  public void update ( Graphics g )
  {
      paintComponent ( g );
  }
  
  @Override
  public void run ()
  {
     while ( Thread.currentThread() == sim )
     {
         repaint ();
         try
         {
             Thread.sleep ( dt );
         }
         catch ( InterruptedException e )
         {
             System.out.println ( "Exception: " + e.getMessage() );
         }        
     }
  }

  public static void main ( String[] args )
  {
      JFrame frame = new JFrame ( "Drivetrain Simulation" );
      Animation panel = new Animation ( 1000, 1000 );
      frame.getContentPane().add ( panel );
      frame.setSize ( panel.width, panel.height );
      frame.setVisible ( true );
      frame.addWindowListener ( new WindowAdapter() {
          public void windowClosing ( WindowEvent evt ) {
              System.exit ( 0 );
          }
      } );
  }
}