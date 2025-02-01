package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeInternal;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@TeleOp
public class QualOpModeV2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        Robot robot = new Robot(hardwareMap, false);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        // init position for intake claw pivot and claw open respectively
        robot.reset();
        robot.read();
        robot.intake.setRotate(IntakeSubsystem.ROTATE_DOWN);
        robot.intake.setClaw();
        robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // gamepad 1. y = yellow, b = red, a = green, x = blue
            if(gamepad1.y){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setRotate(IntakeSubsystem.ROTATE_UP)));
            }

            if(gamepad1.b){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setRotate(IntakeSubsystem.ROTATE_ENTER)));
            }

            if(gamepad1.a){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setRotate(IntakeSubsystem.ROTATE_DOWN)));
            }

//            if(gamepad1.x){
//                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setClaw()));
//            }

            if(gamepad1.dpad_left){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setClaw(IntakeSubsystem.CLAW_CLOSE)));
            }

            if(gamepad1.dpad_right){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setClaw(IntakeSubsystem.CLAW_OPEN)));
            }

//            if(gamepad1.dpad_up){
//                CommandScheduler.getInstance().schedule(new LiftPositionCommand(robot.h_lift, 20, 2, 100, 100));
//            }
//
//            if(gamepad1.dpad_down){
//                CommandScheduler.getInstance().schedule(new LiftPositionCommand(robot.h_lift, -2, 2, 100, 100));
//            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("FrontLeft: ", frontLeftPower);
            telemetry.addData("BackLeft: ", backLeftPower);
            telemetry.addData("FrontRight: ", frontRightPower);
            telemetry.addData("BackRight: ", backRightPower);
            telemetry.update();
            // gamepad 2. y = yellow, b = red, a = green, x = blue

            if(gamepad2.y){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN)));
            }

            if(gamepad2.b){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT)));
            }

            if(gamepad2.a){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT)));
            }

            if(gamepad2.x){
                // Arm pickup position is too far, might need to hold.
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_SPECIMEN)));
            }

            if(gamepad2.dpad_left){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE)));
            }

            if(gamepad2.dpad_up){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP)));
            }

            if(gamepad2.dpad_down){
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_SCORE)));

            }


            CommandScheduler.getInstance().run();
            robot.write();
            robot.loop();
        }
    }
}