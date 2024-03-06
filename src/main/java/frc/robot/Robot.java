// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.commands.DriveForward;
import frc.robot.subsystems.drive.Drive;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.REVPhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private final PS4Controller m_controller = new PS4Controller(0);

  private final Timer m_timer = new Timer();
  public final Drive m_drive = new Drive(false, m_timer); // normalde false
  private double current_time;
  private Command m_autonomousCommand;
  Thread m_visionThread;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    /*
     * m_visionThread = new Thread(
     * () -> {
     * // Get the UsbCamera from CameraServer
     * UsbCamera camera = CameraServer.startAutomaticCapture();
     * // Set the resolution
     * camera.setResolution(640, 480);
     * 
     * // Get a CvSink. This will capture Mats from the camera
     * CvSink cvSink = CameraServer.getVideo();
     * // Setup a CvSource. This will send images back to the Dashboard
     * CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
     * 
     * // Mats are very memory expensive. Lets reuse this Mat.
     * Mat mat = new Mat();
     * 
     * // This cannot be 'true'. The program will never exit if it is. This
     * // lets the robot stop this thread when restarting robot code or
     * // deploying.
     * while (!Thread.interrupted()) {
     * // Tell the CvSink to grab a frame from the camera and put it
     * // in the source mat. If there is an error notify the output.
     * if (cvSink.grabFrame(mat) == 0) {
     * // Send the output the error.
     * outputStream.notifyError(cvSink.getError());
     * // skip the rest of the current iteration
     * continue;
     * }
     * // Put a rectangle on the image
     * Imgproc.rectangle(
     * mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
     * // Give the output stream a new image to display
     * outputStream.putFrame(mat);
     * }
     * });
     */

  }

  @Override
  public void robotPeriodic() {
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    m_drive.idleBrake();
    m_drive.moveIntakeArmUntilSwitchInit();
    m_drive.moveIntakeArmUntilSwitch();
    m_drive.miniArmMovement();
    m_drive.shooterMovement(false);
    current_time = secs();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (secs() - current_time < 2) {
      m_drive.drive(0.5, 0);
    } else {
      m_drive.stop();
    }
  }

  @Override
  public void simulationInit() {
    m_drive.simInit();
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    m_timer.reset();
    m_timer.start();

    m_drive.idleCoast();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } // aman diyim otonom bitince hiç gerek yok böyle şeylere...

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_drive.drive(-m_controller.getLeftY() * 0.5, -m_controller.getLeftX() * 0.5);
    // System.out.println("x:" + m_controller.getLeftY());
    // m_drive.intakeMovement();
    if (m_controller.getTriangleButton()) {
      double start = secs();
      m_drive.intakeMovement(false);
    } else if (m_controller.getCircleButton()) {
      m_drive.shooterMovement(false);
      double currentSecs = secs();
      while (secs() - currentSecs < 0.3)
        ;
      m_drive.intakeMovement(true);
    } else if (m_controller.getCrossButton()) {
      m_drive.intakeMovement(true);
    } else if (m_controller.getSquareButton()) {
      m_drive.shooterMovement(true);
    } else if (m_controller.getR1Button()) {
      m_drive.moveIntakeArmUntilSwitchInit();
    } else if (m_controller.getPOV() == 0) {
      m_drive.moveHumanPlayer(0.33);
    } else if (m_controller.getPOV() == 180) {
      m_drive.moveHumanPlayer(-1);
    } else {
      m_drive.stopIntakeShootAndHumanplayer();
    }
    if (m_controller.getR2Axis() > 0.125) {
      m_drive.moveIntakeArm(0.2);
    } else if (m_controller.getL2Axis() > 0.125) {
      m_drive.moveIntakeArm(-0.2);
    } else {
      m_drive.stopIntakeArm();
    }

    if (!m_drive.getMovementCompleted())
      m_drive.moveIntakeArmUntilSwitch();

    m_drive.updatePose();

    SmartDashboard.putData("field", m_drive.getField());
    SmartDashboard.putNumber("gyro", m_drive.getGyro().getDegrees());
    SmartDashboard.putNumber("intake arm position", m_drive.getIntakeArmPosition());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    m_timer.reset();
    m_timer.start();
  }

  public double secs() {
    return m_timer.get();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // m_drive.drive(0, 0.25);

  }

}
