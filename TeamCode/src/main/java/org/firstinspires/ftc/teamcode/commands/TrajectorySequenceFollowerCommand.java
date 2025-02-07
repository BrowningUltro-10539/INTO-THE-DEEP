package org.firstinspires.ftc.teamcode.commands.Auto;



import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TrajectorySequenceFollowerCommand extends CommandBase {

    private final SampleMecanumDrive drive;
    private final TrajectorySequence trajectory;

    public TrajectorySequenceFollowerCommand(SampleMecanumDrive drive, TrajectorySequence trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.setMotorPowers(0,0,0,0);
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}