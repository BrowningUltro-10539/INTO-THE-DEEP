package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Joystick Strafe Test", group="Test")
public class StrafeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        // Reverse right-side motors if necessary
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double strafePower = gamepad1.left_stick_x; // Left joystick X-axis controls strafing

            // Apply power for strafing
            frontLeftMotor.setPower(strafePower);
            backLeftMotor.setPower(strafePower);
            frontRightMotor.setPower(strafePower);
            backRightMotor.setPower(strafePower);

            // Telemetry for debugging
            telemetry.addData("FrontLeft: ", strafePower);
            telemetry.addData("BackLeft: ", strafePower);
            telemetry.addData("FrontRight: ", strafePower);
            telemetry.addData("BackRight: ", strafePower);
            telemetry.update();
        }

        // Stop all motors when OpMode ends
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
