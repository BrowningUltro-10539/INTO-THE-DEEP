package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeInternal;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class QualOpModeV2FieldCentric extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

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

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        //Since our control hub is mounted on the backplate sideways, the logo is facing forward and the usb facing up.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();
        // init position for intake claw pivot and claw open respectively
        robot.reset();
        robot.read();
//        robot.intake.setRotate(IntakeSubsystem.ROTATE_DOWN);
//        robot.intake.setClaw();
//        robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP);

        

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.left_bumper) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//                    + Math.PI / 2;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing, needs to be tested driving

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


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

            if(gamepad1.dpad_up){
                CommandScheduler.getInstance().schedule(new LiftPositionCommand(robot.h_lift, 10, 2, 100, 100));
            }

            if(gamepad1.dpad_down){
                CommandScheduler.getInstance().schedule(new LiftPositionCommand(robot.h_lift, 0, 2, 100, 100));
            }

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