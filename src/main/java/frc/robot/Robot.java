// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

  private final CommandXboxController m_controller = new CommandXboxController(0);

  private final CANSparkMax m_rearLeft = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_rearRight = new CANSparkMax(2, MotorType.kBrushless);

  private final CANSparkMax m_frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_frontRight = new CANSparkMax(4, MotorType.kBrushless);

  private final CANcoder m_leftEncoder = new CANcoder(6);
  private final CANcoder m_rightEncoder = new CANcoder(5);

  // private DifferentialDrive drive = new DifferentialDrive(, );

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS1);
  private Pose2d m_pose = new Pose2d(5.0, 13.5, new Rotation2d());

  private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
  private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;

  String trajectoryJSON = "output/Unnamed.wpilib.json";
  Trajectory trajectory = new Trajectory();
  public final Drive m_drive = new Drive(m_rearLeft, m_rearRight, m_frontLeft, m_frontRight, true); // normalde true

  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_drive.init();
    DataLogManager.start();
    m_gyro.calibrate();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
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
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_drive.drive(m_controller.getLeftY(), m_controller.getRightX());

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    m_timer.reset();
    m_timer.start();
    // m_drive.drive(0, 0.25);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    /*
     * m_pose = m_odometry.update(m_gyro.getRotation2d(),
     * m_leftEncoder.getPosition().getValueAsDouble(),
     * m_rightEncoder.getPosition().getValueAsDouble());
     * 
     * DataLogManager.log(m_pose.toString());
     */
    DataLogManager.log("angle" + m_gyro.getAngle());
    DataLogManager.log("left encoder: " + m_leftEncoder.getPosition().getValue());
    DataLogManager.log("right encoder: " + m_rightEncoder.getPosition().getValue());
  }
}
