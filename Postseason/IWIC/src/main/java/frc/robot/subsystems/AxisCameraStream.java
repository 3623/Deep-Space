/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.Point;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * Add your docs here.
 */
public class AxisCameraStream {

    Thread m_visionThread;

    public AxisCameraStream(){
      m_visionThread = new Thread(() -> {
        // Get the Axis camera from CameraServer
        AxisCamera camera = CameraServer.getInstance().addAxisCamera("axis-camera.local");
        // Set the resolution
        camera.setResolution(320, 240);
        camera.setFPS(15);
  
        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getInstance().getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 320, 240);
  
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
}
