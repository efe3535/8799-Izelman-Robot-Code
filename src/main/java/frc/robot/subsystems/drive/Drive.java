package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

public class Drive extends SubsystemBase {
    public DifferentialDrive m_drive;
    private final CANSparkMax rearLeft = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax rearRight = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax frontLeft = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax frontRight = new CANSparkMax(4, MotorType.kBrushless);

    public boolean isReversed;
    public DifferentialDriveOdometry odometry;
    public ChassisSpeeds speed;
    public SimDeviceSim[] simDevices = new SimDeviceSim[4];

    public Drive(DifferentialDriveOdometry odometry, ChassisSpeeds speed, boolean reverse) {
        this.odometry = odometry;
        this.speed = speed;
        this.isReversed = reverse;

        this.rearLeft.follow(frontLeft);
        this.rearRight.follow(frontRight);

        this.frontLeft.setInverted(isReversed);
        this.rearLeft.setInverted(isReversed);
        this.frontRight.setInverted(!isReversed);
        this.rearRight.setInverted(!isReversed);

        this.m_drive = new DifferentialDrive(frontLeft, frontRight);
        this.m_drive.setDeadband(0.05);
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(frontLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(frontRight, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(rearLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(rearRight, DCMotor.getNEO(1));
        }
    }

    public void feed() {
        this.m_drive.feed();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return speed;
    }

    public void stop() {
        this.m_drive.stopMotor();
    }

    public void chassisSpeedsDrive(ChassisSpeeds chassisSpeeds) {
        m_drive.arcadeDrive(Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
                chassisSpeeds.omegaRadiansPerSecond);
    }

    public Command followPathCommand(String pathName) {
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

    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("8799_Path_1");

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return AutoBuilder.followPath(path);
    }

    public void drive(double leftMotor, double rightMotor) {
        this.m_drive.arcadeDrive(leftMotor, rightMotor);
    }

    public void updateOdometry(Rotation2d rotation, double leftDistance, double rightDistance) {
        this.odometry.update(rotation, leftDistance, rightDistance);
    }

    public void tankDrive(double leftMotor, double rightMotor) {
        this.m_drive.tankDrive(leftMotor, rightMotor);
    }

    public DifferentialDrive getDifferentialDrive() {
        return this.m_drive;
    }
}