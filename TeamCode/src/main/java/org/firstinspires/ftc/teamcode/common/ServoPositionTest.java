package org.firstinspires.ftc.teamcode.common;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Robot;
import java.util.List;


@Config
@TeleOp
public class ServoPositionTest extends OpMode {
    private ServoImplEx servo, leftClaw, rightClaw;
    public static double pos1 = 0, type = 0;

    public static double pos2 = 0.5;
    @Override
    public void init() {
        servo = hardwareMap.get(ServoImplEx.class, "leftArm");
    }
    @Override
    public void loop() {
        if(gamepad1.b) servo.setPosition(pos1);

        if(gamepad2.a){servo.setPosition(pos2);}

    }
}
