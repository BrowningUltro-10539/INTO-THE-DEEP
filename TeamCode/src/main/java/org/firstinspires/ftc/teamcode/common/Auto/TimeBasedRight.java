package org.firstinspires.ftc.teamcode.common.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;


@Autonomous
public class TimeBasedRight extends LinearOpMode {


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


        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // set arm to midpoint continaing preloaded specimen and set intake claw to correct pivot position.
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_SCORE)),
                new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT)),
                new InstantCommand(() -> robot.intake.setRotate(IntakeSubsystem.ROTATE_ENTER))
        ));

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
        // score specimen
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT)),
                new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN))
        ));

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
