// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drive.Drive;
import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
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

  private final XboxController m_controller = new XboxController(0);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final Field2d m_field = new Field2d();
  private final ChassisSpeeds m_speeds = new ChassisSpeeds(2, 0, Math.PI);

  private final CANcoder m_leftEncoder = new CANcoder(6);
  private final CANcoder m_rightEncoder = new CANcoder(5);

  private Pose2d m_pose = new Pose2d(1.88, 7.02, new Rotation2d());

  private final double kDriveTick2Cm = (2 * Math.PI * 3) * 2.54;
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
      m_leftEncoder.getPosition().getValue() * kDriveTick2Cm, m_rightEncoder.getPosition().getValue() * kDriveTick2Cm);

  public final Drive m_drive = new Drive(m_odometry, m_speeds, false); // normalde false

  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    DataLogManager.start();

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    m_drive.stop();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 3 seconds

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    m_timer.reset();
    m_timer.start();

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_odometry.resetPosition(m_gyro.getRotation2d(), new DifferentialDriveWheelPositions(0, 0), m_pose);
    SmartDashboard.putNumber("leftENCODER", m_leftEncoder.getPosition().getValue());
    SmartDashboard.putData("pose2d", m_field);
    SmartDashboard.putNumber("rightENCODER", m_rightEncoder.getPosition().getValue());
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_drive.drive(-m_controller.getLeftY(), -m_controller.getLeftX());
    m_drive.updateOdometry(m_gyro.getRotation2d(), m_leftEncoder.getPosition().getValue() * kDriveTick2Cm,
        m_rightEncoder.getPosition().getValue() * kDriveTick2Cm);
    m_field.setRobotPose(m_pose);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    m_timer.reset();
    m_timer.start();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_odometry.resetPosition(m_gyro.getRotation2d(), new DifferentialDriveWheelPositions(0, 0), m_pose);
    SmartDashboard.putNumber("leftENCODER", m_leftEncoder.getPosition().getValue());
    SmartDashboard.putData("pose2d", m_field);
    SmartDashboard.putNumber("rightENCODER", m_rightEncoder.getPosition().getValue());
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    m_drive.updateOdometry(m_gyro.getRotation2d(), m_leftEncoder.getPosition().getValue() * kDriveTick2Cm,
        m_rightEncoder.getPosition().getValue() * kDriveTick2Cm);
    m_field.setRobotPose(m_pose);
    m_drive.feed();

    /*
     * double leftPosition = m_leftEncoder.getPosition().getValue() * kDriveTick2Cm;
     * double rightPosition = m_rightEncoder.getPosition().getValue() *
     * kDriveTick2Cm;
     * double distance = (leftPosition + rightPosition) / 2;
     * double target = 25;
     * double kError = target - distance;
     * double kP = 1 / target;
     * double kSpeed = kP * kError;
     * // Proportional integral derivative
     * if (m_controller.getAButton()) {
     * 
     * }
     * if (distance < target) {
     * m_drive.drive(kSpeed, 0);
     * } else {
     * m_drive.stop();
     * }
     * DataLogManager.log("mesafe:" + distance);
     */
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
