package org.firstinspires.ftc.teamcode.common.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class TESTPOSITIONOneSpecimenThreeSample extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        final double FORWARD_SPEED = 0.6;
        final double STRAFE_SPEED  = 0.5;

        // Reverse right motors to correct movement direction
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        // Move forward
        runtime.reset();
        drive(FORWARD_SPEED, 3.0);

        // Move back slightly
        drive(-FORWARD_SPEED, 1.0);

        // Move right slightly
        strafe(STRAFE_SPEED, 1.0);

        // Move forward past sample
        drive(FORWARD_SPEED, 2.0);

        // Move right slightly to align with sample
        strafe(STRAFE_SPEED, 1.0);

        // Move back to bring to observation zone
        drive(-FORWARD_SPEED, 2.0);

        // Move forward past second sample
        drive(FORWARD_SPEED, 2.0);

        // Move right to align with second sample
        strafe(STRAFE_SPEED, 1.0);

        // Move back to bring second sample to observation zone
        drive(-FORWARD_SPEED, 2.0);

        // Move forward past third sample
        drive(FORWARD_SPEED, 2.0);

        // Move right to align with third sample
        strafe(STRAFE_SPEED, 1.0);

        // Move back to bring third sample to observation zone
        drive(-FORWARD_SPEED, 2.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    public void drive(double speed, double time) {
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        sleep((long) (time * 1000));
        stopMotors();
    }

    public void strafe(double speed, double time) {
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(speed);
        sleep((long) (time * 1000));
        stopMotors();
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
