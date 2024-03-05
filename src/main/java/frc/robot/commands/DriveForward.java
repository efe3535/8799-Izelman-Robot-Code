package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForward extends Command {
    private final DifferentialDrive drive;
    private final double time;
    private final Timer m_timer;
    private double currentTime;
    private double turn;

    public double secs() {
        return m_timer.get();
    }

    public DriveForward(DifferentialDrive drive, double seconds, Timer timer, double turn) {
        this.drive = drive;
        this.time = seconds;
        this.m_timer = timer;
        this.turn = turn;
    }

    @Override
    public void initialize() {
        System.out.println("DriveForwardCmd started!");
        this.currentTime = secs();
    }

    @Override
    public void execute() {
        double time = secs();
        this.drive.arcadeDrive(0.75, this.turn);
        while (secs() - currentTime < time)
            ;

        this.drive.stopMotor();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopMotor();
    }

    @Override
    public boolean isFinished() {
        if (secs() - currentTime > this.time)
            return true;
        else
            return false;
    }
}