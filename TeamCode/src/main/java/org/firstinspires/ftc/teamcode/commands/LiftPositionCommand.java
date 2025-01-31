package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalLiftSubsystem;

public class LiftPositionCommand extends CommandBase {
    private final double position;
    private final double allowed_error, max_v, max_a;

    private final HorizontalLiftSubsystem lift;

    public LiftPositionCommand(HorizontalLiftSubsystem lift, double position) {
        this.position = position;
        this.lift = lift;
    }

    @Override
    public void execute() {
//
//        lift.newProfile(position, max_v, max_a);
     lift.setTargetLiftPosition(position);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(lift.getLiftPosition() - position) < allowed_error;
    }
}