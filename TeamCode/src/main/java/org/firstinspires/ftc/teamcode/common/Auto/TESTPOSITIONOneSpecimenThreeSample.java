package org.firstinspires.ftc.teamcode.common.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@Autonomous
public class TESTPOSITIONOneSpecimenThreeSample extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        Robot robot = new Robot(hardwareMap, true);
        robot.reset();
        robot.outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE);
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        final double FORWARD_SPEED = 0.4;
        final double STRAFE_SPEED = 0.4;
        final double TURN_SPEED = 0.255; // Speed for turning

        // Reverse right motors to correct movement direction
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Move forward
        runtime.reset();
        strafe(-STRAFE_SPEED, 0.375);
        robot.intake.setRotate(IntakeSubsystem.ROTATE_ENTER);
        robot.outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE);
        robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT);
        robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP);
        drive(FORWARD_SPEED, 0.95);
        sleep(750);
        robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT);
        sleep(1500); // before it was at 2000
        robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN);
        sleep(150);
        drive(FORWARD_SPEED, 0.35);
        sleep(500);
        // Move back slightly
        drive(-FORWARD_SPEED, 0.3);
        sleep(500);
        // Move right slightly
        strafe(STRAFE_SPEED, 1.7);
        sleep(500);
        // Move forward past sample
        drive(FORWARD_SPEED, 0.8);
        sleep(500);
        // Move right slightly to align with sample
        strafe(STRAFE_SPEED, 0.75);
        sleep(500);
        // Move back to bring to observation zone
        drive(-FORWARD_SPEED, 1.3);
        robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_SPECIMEN);
        turn(180, TURN_SPEED);
        robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP);
        sleep(1500);
        robot.outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE);

        // Example usage of the turn method
     // Turn 90 degrees counterclockwise

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

    /**
     * Turns the robot by a specified angle.
     *
     * @param angleDegrees The angle to turn in degrees. Positive values turn clockwise, negative values turn counterclockwise.
     * @param speed        The speed at which to turn (0.0 to 1.0).
     */
    public void turn(double angleDegrees, double speed) {
        // Calculate the time required to turn based on the angle and speed
        double degreesPerSecond = 90; // Adjust this value based on your robot's turning speed
        double turnTime = Math.abs(angleDegrees) / degreesPerSecond;

        // Set motor powers for turning
        if (angleDegrees > 0) {
            // Clockwise turn
            frontLeftMotor.setPower(speed);
            backLeftMotor.setPower(speed);
            frontRightMotor.setPower(-speed);
            backRightMotor.setPower(-speed);
        } else {
            // Counterclockwise turn
            frontLeftMotor.setPower(-speed);
            backLeftMotor.setPower(-speed);
            frontRightMotor.setPower(speed);
            backRightMotor.setPower(speed);
        }

        // Wait for the turn to complete
        sleep((long) (turnTime * 1000));

        // Stop motors after turning
        stopMotors();
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}