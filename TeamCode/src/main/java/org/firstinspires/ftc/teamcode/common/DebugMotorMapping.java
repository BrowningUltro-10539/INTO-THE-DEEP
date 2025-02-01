package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Debug Motor Mapping", group="Test")
public class DebugMotorMapping extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        // Wait for start signal
        waitForStart();

        while (opModeIsActive()) {
            // Each joystick controls one wheel
            double frontLeftPower = -gamepad1.left_stick_y;   // Gamepad 1 Left Stick Y
            double backLeftPower = -gamepad1.right_stick_y;   // Gamepad 1 Right Stick Y
            double frontRightPower = -gamepad2.left_stick_y;  // Gamepad 2 Left Stick Y
            double backRightPower = -gamepad2.right_stick_y;  // Gamepad 2 Right Stick Y

            // Set motor power
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Display which motor is moving
            telemetry.addData("Front Left Power (GP1 Left Stick Y):", frontLeftPower);
            telemetry.addData("Back Left Power (GP1 Right Stick Y):", backLeftPower);
            telemetry.addData("Front Right Power (GP2 Left Stick Y):", frontRightPower);
            telemetry.addData("Back Right Power (GP2 Right Stick Y):", backRightPower);
            telemetry.update();
        }
    }
}
