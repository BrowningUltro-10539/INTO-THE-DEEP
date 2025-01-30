package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand2;

@Config
@TeleOp
public class SlideTestOpMode extends OpMode {
    Robot robot;
    public double targetPosition = 0;
    @Override
    public void init(){
        robot = new Robot(hardwareMap, false);
    }
    @Override
    public void loop(){
        CommandScheduler.getInstance().schedule(
                new LiftPositionCommand2(robot.v_lift, targetPosition, 2, 200, 2)
        );
    }
}
