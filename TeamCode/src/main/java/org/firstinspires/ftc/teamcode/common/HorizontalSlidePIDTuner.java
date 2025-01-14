package org.firstinspires.ftc.teamcode.common;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Robot;
import java.util.List;


@Config
@TeleOp


public class HorizontalSlidePIDTuner extends OpMode {


    public DcMotorEx liftMotorOne;
    public DcMotorEx liftEncoder;
    public PIDController liftController;


    private Robot robot;


    public static double slideP = 0;
    public static double slideI = 0;
    public static double slideD = 0;
    public static double slideKg = 0;
    public static double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 0.764445002 / 145.1;
    public static double targetPosition = 0;
    public List<LynxModule> controllers;

    @Override
    public void init(){
        liftController = new PIDController(slideP, slideI, slideD);
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "horizontalLift");
        liftMotorOne.setDirection(DcMotorSimple.Direction.REVERSE); // !!!!
        controllers = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : controllers){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }
    @Override
    public void loop(){
        liftController.setPID(slideP, slideI, slideD);
        double liftPosition = liftMotorOne.getCurrentPosition() * SLIDE_TICKS_PER_INCH;
        double liftPower = liftController.calculate(liftPosition, targetPosition);

        liftMotorOne.setPower(liftPower);

        telemetry.addData("Lift Position, Motor 1", liftPosition);
        telemetry.addLine();

        telemetry.addData("Lift Target", targetPosition);
        telemetry.addData("Lift Power", liftPower);
        telemetry.update();

        for(LynxModule module : controllers) {
            module.clearBulkCache();
        }
    }
}
