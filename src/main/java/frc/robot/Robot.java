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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.drive.Drive;

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
  private double first;
  private final Timer m_timer = new Timer();
  public final Drive m_drive = new Drive(false, m_timer); // normalde false
  private double gyroDegrees = m_drive.gyroDegrees();
  private final PIDController m_gyroController = new PIDController(0.01, 0, 0);
  public double timerStart;

  private Command m_autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
  }

  private final boolean ortadayiz = true;

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    // timerStart = m_timer.get();

    m_drive.shooterMovement(false);

    if (m_timer.hasElapsed(1.5)) {
      m_drive.intakeMovement(true);
    }

    if (m_timer.hasElapsed(3)) {
      m_drive.stopIntakeShootAndHumanplayer();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

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
    m_drive.drive(-m_controller.getLeftY() * 1, -m_controller.getLeftX() * 1);
    // System.out.println("x:" + m_controller.getLeftY());
    // m_drive.intakeMovement();
    if (m_controller.getTriangleButton()) {
      double start = secs();
      m_drive.intakeMovement(false);
    } else if (m_controller.getCircleButton()) {
      m_drive.shooterMovement(false);
      double currentSecs = secs();
      while (secs() - currentSecs < 0.5) // eskiden 0.3
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

    if (!m_drive.getMovementCompleted() && m_drive.getIntakeArmInit()) // Bi kontrol et getIntakeArmInit'i
      m_drive.moveIntakeArmUntilSwitch();

    m_drive.updatePose();

    SmartDashboard.putData("field", m_drive.getField());
    SmartDashboard.putNumber("odo", m_drive.getDistance());

    SmartDashboard.putNumber("gyro", m_drive.getGyro().getRotation2d().getDegrees());
    SmartDashboard.putNumber("intake arm position", m_drive.getIntakeArmPosition());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    m_timer.reset();
    m_drive.idleBrake();
    m_timer.start();

    m_gyroController.enableContinuousInput(-180, 180);
    m_gyroController.setTolerance(3, 5);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_gyroController.calculate(m_drive.getGyro().getAngle(), 30);
    if (!m_gyroController.atSetpoint()) {
      m_drive.drive(0, m_gyroController.calculate(m_drive.getGyro().getAngle(), 30));

    }
  }

  public double secs() {
    return m_timer.get();
  }

}
