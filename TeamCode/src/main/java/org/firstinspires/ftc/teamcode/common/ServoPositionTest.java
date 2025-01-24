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
    public static double pos = 0, type = 0;
    @Override
    public void init() {
        servo = hardwareMap.get(ServoImplEx.class, "leftServo");
    }
    @Override
    public void loop() {
        if(type == 0) servo.setPosition(pos);

    }
}
