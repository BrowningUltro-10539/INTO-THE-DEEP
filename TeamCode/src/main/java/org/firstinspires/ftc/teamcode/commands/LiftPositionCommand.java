package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalLiftSubsystem;

public class LiftPositionCommand extends CommandBase {
    private final double position;
    private final double allowed_error, max_v, max_a;

    private final HorizontalLiftSubsystem lift;

    public LiftPositionCommand(HorizontalLiftSubsystem lift, double position, double allowed_error, double max_a, double max_v) {
        this.position = position;
        this.lift = lift;
        this.allowed_error = allowed_error;
        this.max_v = max_v;
        this.max_a = max_a;
    }

    @Override
    public void execute() {
        lift.setTargetLiftPosition(position);
//        lift.newProfile(position, max_v, max_a);
        /*
        I'll consider re-implementing when I have a better idea of how to tune / control motion profiling
         */
    }

    @Override
    public boolean isFinished() {
        return Math.abs(lift.getLiftPosition() - position) < allowed_error;
    }
}