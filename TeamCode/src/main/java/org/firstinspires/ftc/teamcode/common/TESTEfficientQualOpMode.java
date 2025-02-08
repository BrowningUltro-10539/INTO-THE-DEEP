package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeInternal;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp
public class TESTEfficientQualOpMode extends LinearOpMode {
//    public static double slideP = 0.13;
//    public static double slideI = 0;
//    public static double slideD = 0;
//    public static double slideKg = 0;
//    public static double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 0.764445002 / 384.5;
//    public static double targetPosition = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, false);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        DcMotorEx liftMotorOne = hardwareMap.get(DcMotorEx.class, "horizontalMotor");
//        liftMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
//        PIDController liftController = new PIDController(slideP, slideI, slideD);

        List<LynxModule> controllers = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule module : controllers) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            for (LynxModule module : controllers) {
//                module.clearBulkCache(); // Clear bulk cache BEFORE reading
//            }

            robot.read();

            // PID control for slides
//            double liftPosition = liftMotorOne.getCurrentPosition() * SLIDE_TICKS_PER_INCH;
//            double liftPower = liftController.calculate(liftPosition, targetPosition);
//            liftMotorOne.setPower(liftPower);

            // Read IMU heading inside loop for real-time field-centric control
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Get gamepad inputs
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double motorPower = -gamepad2.left_stick_y;

            if (gamepad1.left_bumper) imu.resetYaw();

            // Field-centric control
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftMotor.setPower((rotY + rotX + rx) / denominator);
            backLeftMotor.setPower((rotY - rotX + rx) / denominator);
            frontRightMotor.setPower((rotY - rotX - rx) / denominator);
            backRightMotor.setPower((rotY + rotX - rx) / denominator);

            liftMotorOne.setPower(motorPower);

            // Optimize button press actions
            if (gamepad1.y) robot.intake.setRotate(IntakeSubsystem.ROTATE_UP);
            if (gamepad1.b) robot.intake.setRotate(IntakeSubsystem.ROTATE_ENTER);
            if (gamepad1.a) robot.intake.setRotate(IntakeSubsystem.ROTATE_DOWN);
            if (gamepad1.dpad_left) robot.intake.setClaw(IntakeSubsystem.CLAW_CLOSE);
            if (gamepad1.dpad_right) robot.intake.setClaw(IntakeSubsystem.CLAW_OPEN);
//            if (gamepad1.dpad_up) targetPosition = 25;
//            if (gamepad1.dpad_down) ;

            if (gamepad2.y) robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN);
            if (gamepad2.b) robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT);
            if (gamepad2.a) robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT);
            if (gamepad2.x) robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_SPECIMEN);
            if (gamepad2.dpad_left) robot.outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE);
            if (gamepad2.dpad_up) robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP);
            if (gamepad2.dpad_down) robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_SCORE);

            // Run subsystems
            CommandScheduler.getInstance().run();
            robot.write();
            robot.intake.loop();
            robot.outtake.loop();
        }
    }
}
