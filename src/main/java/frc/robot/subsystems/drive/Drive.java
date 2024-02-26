package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;

public class Drive extends SubsystemBase {
    public DifferentialDrive m_drive;
    public CANSparkMax rearLeft, rearRight, frontLeft, frontRight;
    public boolean isReversed;

    public Drive(CANSparkMax rearLeft, CANSparkMax rearRight, CANSparkMax frontLeft, CANSparkMax frontRight,
            boolean reverse) {
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.isReversed = reverse;
    }

    public void test() {
        frontLeft.set(0.25);
        rearLeft.set(0.25);
    }

    public void init() {
        rearLeft.follow(frontLeft);
        rearRight.follow(frontRight);

        frontLeft.setInverted(isReversed);
        rearLeft.setInverted(isReversed);
        frontRight.setInverted(!isReversed);
        rearRight.setInverted(!isReversed);

        this.m_drive = new DifferentialDrive(frontLeft, frontRight);
    }

    public void drive(double leftMotor, double rightMotor) {
        this.m_drive.arcadeDrive(leftMotor * 1, rightMotor * 1);
    }

    public void tankDrive(double leftMotor, double rightMotor) {
        this.m_drive.tankDrive(leftMotor, rightMotor);
    }

    public DifferentialDrive getDifferentialDrive() {
        return this.m_drive;
    }
}