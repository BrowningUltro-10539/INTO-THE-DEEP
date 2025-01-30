package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@Config
@TeleOp
public class SlideTestOpMode extends OpMode {
    Robot robot;
    @Override
    public void init(){
        robot = new Robot(hardwareMap, false);
    }
    @Override
    public void loop(){
        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT))
        );
    }
}
