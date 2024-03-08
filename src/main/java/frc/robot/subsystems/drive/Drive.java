package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveForward;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.IdleMode;

public class Drive extends SubsystemBase {
    public DifferentialDrive m_drive;
    private final CANSparkMax rearLeft = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax frontRight = new CANSparkMax(4, MotorType.kBrushless);
    private final CANSparkMax frontLeft = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax rearRight = new CANSparkMax(2, MotorType.kBrushless);

    private final CANSparkMax intakeMotor = new CANSparkMax(6, MotorType.kBrushless);
    private boolean completedMovement = false;
    private boolean armMovementInit = false;
    private final CANSparkMax intakeArmMotor = new CANSparkMax(5, MotorType.kBrushless);

    private final WPI_VictorSPX leftShooter = new WPI_VictorSPX(9);
    private final WPI_VictorSPX humanPlayerTake = new WPI_VictorSPX(7);
    private final WPI_VictorSPX rightShooter = new WPI_VictorSPX(10);
    private final DigitalInput armLimitSwitch = new DigitalInput(0);
    public boolean isReversed;
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private final Field2d m_field = new Field2d();
    private final ChassisSpeeds m_speeds = new ChassisSpeeds(0, 0, Math.PI);
    private final CANcoder m_leftEncoder = new CANcoder(12);
    private final CANcoder m_rightEncoder = new CANcoder(11);
    private final Timer m_timer;
    private Pose2d m_pose = new Pose2d(1.88, 7.02, new Rotation2d());
    private final double kDriveTick2Meters = 2.54 * 6 * Math.PI / 4096 * 100;
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
            m_leftEncoder.getPosition().getValue() * kDriveTick2Meters,
            m_rightEncoder.getPosition().getValue() * kDriveTick2Meters);

    private final double maxIntakeArmDelta = 5.936;

    public void updatePose() {
        this.updateOdometry(m_gyro.getRotation2d(), m_leftEncoder.getPosition().getValue() * kDriveTick2Meters,
                m_rightEncoder.getPosition().getValue() * kDriveTick2Meters);
        m_field.setRobotPose(this.m_odometry.getPoseMeters());
    }

    public Rotation2d getGyro() {
        return m_gyro.getRotation2d();
    }

    public Field2d getField() {
        return m_field;
    }

    public void simInit() {

        REVPhysicsSim.getInstance().addSparkMax(frontLeft, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(frontRight, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(rearRight, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(rearLeft, DCMotor.getNEO(1));
    }

    public void testDistance() {
        double leftPosition = m_leftEncoder.getPosition().getValue() *
                kDriveTick2Meters;
        double rightPosition = m_rightEncoder.getPosition().getValue() *
                kDriveTick2Meters;
        double distance = (leftPosition + rightPosition) / 2;

    }

    public void idleCoast() {
        this.frontLeft.setIdleMode(IdleMode.kCoast);
        this.rearLeft.setIdleMode(IdleMode.kCoast);
        this.frontRight.setIdleMode(IdleMode.kCoast);
        this.rearRight.setIdleMode(IdleMode.kCoast);
    }

    public void idleBrake() {
        this.frontLeft.setIdleMode(IdleMode.kBrake);
        this.rearLeft.setIdleMode(IdleMode.kBrake);
        this.frontRight.setIdleMode(IdleMode.kBrake);
        this.rearRight.setIdleMode(IdleMode.kBrake);
    }

    public void moveHumanPlayer(double speed) {
        humanPlayerTake.set(speed);
    }

    public double getDistance() {
        return (m_leftEncoder.getPosition().getValue() * kDriveTick2Meters
                + m_rightEncoder.getPosition().getValue() * kDriveTick2Meters) / 2;
    }

    public void forwardMeters(double meters) {
        double currentMeters = getDistance();
        this.drive(0.6, 0);
        while (getDistance() < currentMeters + meters)
            ;
        this.stop();

    }

    public Drive(boolean reverse, Timer timer) {

        this.m_timer = timer;

        this.frontLeft.setIdleMode(IdleMode.kCoast);
        this.rearLeft.setIdleMode(IdleMode.kCoast);
        this.frontRight.setIdleMode(IdleMode.kCoast);
        this.rearRight.setIdleMode(IdleMode.kCoast);

        this.frontLeft.setInverted(reverse);
        this.rearLeft.setInverted(reverse);
        this.frontRight.setInverted(!reverse);
        this.rearRight.setInverted(!reverse);

        this.rearLeft.follow(frontLeft);
        this.rearRight.follow(frontRight);

        this.m_drive = new DifferentialDrive(frontLeft, frontRight);
        this.m_drive.setDeadband(0.15);
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
        m_odometry.resetPosition(m_gyro.getRotation2d(), new DifferentialDriveWheelPositions(0, 0), m_pose);

    }

    public void resetEncoders() {

        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public void feed() {
        this.m_drive.feed();

    }

    public void putval() {
        SmartDashboard.putData("pose2d", m_field);
        SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
        double leftPosition = m_leftEncoder.getPosition().getValue() *
                kDriveTick2Meters;
        double rightPosition = m_rightEncoder.getPosition().getValue() *
                kDriveTick2Meters;
        double distance = (leftPosition + rightPosition) / 2;
        double target = 1;
        double kError = target - distance;
        double kP = 1 / distance;

        double[] kArray = { distance, kError, kP, kP * kError };
        SmartDashboard.putNumberArray("values", kArray);
    }

    public void pidTest() {
        double leftPosition = m_leftEncoder.getPosition().getValue() *
                kDriveTick2Meters;
        double rightPosition = m_rightEncoder.getPosition().getValue() *
                kDriveTick2Meters;
        double distance = (leftPosition + rightPosition) / 2;
        double target = 1;
        double kError = target - distance;
        double kP = 1 / distance;

        // double speed1 = m_pidController.calculate(distance, 1);

        this.drive(kP * kError, 0);
    }

    public void intakeMovement(boolean reverse) {
        // pid kontrol yap hocam
        intakeMotor.set(reverse ? 1 : -0.3);

    }

    public void moveIntakeArmUntilSwitchInit() {
        completedMovement = false;
        armMovementInit = true;
    }

    public boolean getIntakeArmInit() {
        return armMovementInit;
    }

    public void delay(double seconds) {
        double time = m_timer.get();
        while (m_timer.get() < time + seconds)
            ;
    }

    public void moveIntakeArmUntilSwitch() {
        if (getLimitSwitchOn() && !completedMovement) {
            intakeArmMotor.set(-0.22);
        } else {
            intakeArmMotor.stopMotor();
            delay(0.5);
            miniArmMovement();
            completedMovement = true;
        }

    }

    public void moveIntakeArmUntilEncoder() {
        double currentEncoder = intakeArmMotor.getEncoder().getPosition();
        double target = currentEncoder + maxIntakeArmDelta;
        moveIntakeArm(0.2);
        while (intakeArmMotor.getEncoder().getPosition() < target)
            ;

        stopIntakeArm();
    }

    public void moveIntakeArmBlocking() {
        intakeArmMotor.set(-0.22);
        while (getLimitSwitchOn())
            ;
        intakeArmMotor.stopMotor();
        delay(0.5);
        miniArmMovement();
    }

    public void miniArmMovement() {
        double armEncoder = intakeArmMotor.getEncoder().getPosition();
        intakeArmMotor.set(0.3);
        while (intakeArmMotor.getEncoder().getPosition() < armEncoder + 1.6)
            ;
        intakeArmMotor.stopMotor();
    }

    public boolean getMovementCompleted() {
        return completedMovement;
    }

    public void shooterMovement(boolean reverse) {
        leftShooter.set(reverse ? -0.33 : 0.75);
        rightShooter.set(reverse ? 0.33 : -0.75);

    }

    public void shooterMovementAuto(boolean reverse) {
        leftShooter.set(reverse ? -0.33 : 0.75);
        rightShooter.set(reverse ? 0.33 : -0.75);

        delay(1.5);
        leftShooter.stopMotor();
        rightShooter.stopMotor();

    }

    public void stopIntakeShootAndHumanplayer() {
        intakeMotor.stopMotor();
        leftShooter.stopMotor();
        rightShooter.stopMotor();
        humanPlayerTake.stopMotor();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public ChassisSpeeds getCurrentSpeeds() {
        SmartDashboard.putString("chassisSpeeds", m_speeds.toString());
        return m_speeds;
    }

    public void stop() {
        this.m_drive.stopMotor();
    }

    public void chassisSpeedsDrive(ChassisSpeeds chassisSpeeds) {
        // SmartDashboard.putNumber("autospeed",
        // Math.hypot(chassisSpeeds.vxMetersPerSecond,
        // chassisSpeeds.vyMetersPerSecond));
        // SmartDashboard.putNumber("autorotation",
        // chassisSpeeds.omegaRadiansPerSecond);

        double[] arr = { chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond };

        SmartDashboard.putNumberArray("wheel speeds", arr);

        /*
         * m_drive.arcadeDrive(Math.hypot(chassisSpeeds.vxMetersPerSecond,
         * chassisSpeeds.vyMetersPerSecond),
         * chassisSpeeds.omegaRadiansPerSecond);
         */
    }

    public void moveIntakeArm(double speed) {
        intakeArmMotor.set(speed);
    }

    public void stopIntakeArm() {
        intakeArmMotor.stopMotor();
    }

    public boolean getLimitSwitchOn() {
        return armLimitSwitch.get();
    }

    public double secs() {
        return m_timer.get();
    }

    public void turnDegreesCCW(double degrees) {
        double gyroDegrees = m_gyro.getRotation2d().getDegrees();

        m_drive.arcadeDrive(0, 0.1);
        if (m_gyro.getRotation2d().getDegrees() >= gyroDegrees + degrees) {
            m_drive.stopMotor();
        }
    }

    public double gyroDegrees() {
        return m_gyro.getRotation2d().getDegrees();
    }

    public void turnDegreesCW(double degrees) {
        double gyroDegrees = m_gyro.getRotation2d().getDegrees();

        while (m_gyro.getRotation2d().getDegrees() > gyroDegrees - degrees) {
            m_drive.arcadeDrive(0, -0.4);
        }
        m_drive.stopMotor();
    }

    public double getIntakeArmPosition() {
        return intakeArmMotor.getEncoder().getPosition();
    }

    public void resetIntakeArmEncoder() {
        intakeArmMotor.getEncoder().setPosition(0);
    }

    public Command getAutonomousCommand() {
        Command auto = new DriveForward(m_drive, 2, m_timer, 0);
        return auto;

    }

    public Command followPathCommand(String pathName) { // Boradan Selamlar
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathRamsete(
                path,
                this::getPose, // Robot pose supplier
                this::getCurrentSpeeds, // Current ChassisSpeeds supplier
                this::chassisSpeedsDrive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void drive(double speed, double rotation) {
        this.m_drive.arcadeDrive(speed, rotation);
    }

    public void updateOdometry(Rotation2d rotation, double leftDistance, double rightDistance) {
        this.m_odometry.update(rotation, leftDistance, rightDistance);
    }

    public void tankDrive(double leftMotor, double rightMotor) {
        this.m_drive.tankDrive(leftMotor, rightMotor);
    }

    public DifferentialDrive getDifferentialDrive() {
        return this.m_drive;
    }
}