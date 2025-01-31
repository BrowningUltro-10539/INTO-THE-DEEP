package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalLiftSubsystem;

public class LiftPositionCommand extends CommandBase {
    private final double position;
    private final double allowed_error, max_v, max_a;

    private final HorizontalLiftSubsystem lift;

    public LiftPositionCommand(HorizontalLiftSubsystem lift, double position, double allowed_error, double v, double a) {
        this.position = position;
        this.allowed_error = allowed_error;
        this.lift = lift;
        this.max_v = v;
        this.max_a = a;
    }

    @Override
    public void execute() {

        lift.newProfile(position, max_v, max_a);
     lift.setTargetLiftPosition(position);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(lift.getLiftPosition() - position) < allowed_error;
    }
}