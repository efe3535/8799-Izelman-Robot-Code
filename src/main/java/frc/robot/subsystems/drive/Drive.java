package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Drive extends SubsystemBase {
    public DifferentialDrive m_drive;
    public WPI_VictorSPX rearLeft, rearRight, frontLeft, frontRight;
    public boolean isReversed;
    public NeutralMode currentMode = NeutralMode.Coast;

    public Drive(WPI_VictorSPX rearLeft, WPI_VictorSPX rearRight, WPI_VictorSPX frontLeft, WPI_VictorSPX frontRight,
            boolean reverse) {
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.isReversed = reverse;
    }

    public Command ToggleBrake() {
        return this.runOnce(() -> this.toggleBrake());
    }

    private void toggleBrake() {
        rearLeft.setNeutralMode(currentMode == NeutralMode.Coast ? NeutralMode.Brake : NeutralMode.Coast);
        rearRight.setNeutralMode(currentMode == NeutralMode.Coast ? NeutralMode.Brake : NeutralMode.Coast);
        frontLeft.setNeutralMode(currentMode == NeutralMode.Coast ? NeutralMode.Brake : NeutralMode.Coast);
        frontRight.setNeutralMode(currentMode == NeutralMode.Coast ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void init() {
        rearLeft.configFactoryDefault();
        frontLeft.configFactoryDefault();
        rearRight.configFactoryDefault();
        frontRight.configFactoryDefault();

        rearLeft.setNeutralMode(currentMode);
        frontLeft.setNeutralMode(currentMode);
        rearRight.setNeutralMode(currentMode);
        frontRight.setNeutralMode(currentMode);

        rearLeft.follow(frontLeft);
        rearRight.follow(frontRight);

        frontLeft.setInverted(isReversed);
        frontRight.setInverted(!isReversed);

        rearLeft.setInverted(InvertType.FollowMaster);
        rearRight.setInverted(InvertType.FollowMaster);

        this.m_drive = new DifferentialDrive(frontLeft, frontRight);
    }

    public void drive(double leftMotor, double rightMotor) {
        this.m_drive.arcadeDrive(leftMotor, rightMotor);
    }

    public DifferentialDrive getDifferentialDrive() {
        return this.m_drive;
    }
}
