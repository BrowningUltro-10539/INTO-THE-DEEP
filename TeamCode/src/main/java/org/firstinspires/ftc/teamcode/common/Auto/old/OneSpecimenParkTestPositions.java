package org.firstinspires.ftc.teamcode.common.Auto.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


@Autonomous
public class OneSpecimenParkTestPositions extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, false);// Use a Pushbot's hardware
        ElapsedTime runtime = new ElapsedTime();

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        final double FORWARD_SPEED = 0.6;
        final double TURN_SPEED    = 0.5;

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // move forward
        runtime.reset();
        frontRightMotor.setPower(FORWARD_SPEED);
        frontLeftMotor.setPower(FORWARD_SPEED);
        backLeftMotor.setPower(FORWARD_SPEED);
        backRightMotor.setPower(FORWARD_SPEED);
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // move backwards
        runtime.reset();
        frontRightMotor.setPower(-FORWARD_SPEED);
        frontLeftMotor.setPower(-FORWARD_SPEED);
        backLeftMotor.setPower(-FORWARD_SPEED);
        backRightMotor.setPower(-FORWARD_SPEED);
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // turn 90 degrees right
        runtime.reset();
        frontRightMotor.setPower(TURN_SPEED);
        frontLeftMotor.setPower(-TURN_SPEED);
        backLeftMotor.setPower(TURN_SPEED);
        backRightMotor.setPower(-TURN_SPEED);
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // move forward to observation zone
        runtime.reset();
        frontRightMotor.setPower(FORWARD_SPEED);
        frontLeftMotor.setPower(FORWARD_SPEED);
        backLeftMotor.setPower(FORWARD_SPEED);
        backRightMotor.setPower(FORWARD_SPEED);
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

}
