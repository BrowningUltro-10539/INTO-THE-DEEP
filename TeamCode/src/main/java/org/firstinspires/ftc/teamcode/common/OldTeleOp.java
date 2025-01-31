//package org.firstinspires.ftc.teamcode.common;
//
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
//
//@TeleOp
//public class OldTeleOp extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Robot robot = new Robot(hardwareMap, false);
//        robot.reset();
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
//
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        imu.initialize(parameters);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//        while (opModeIsActive()) {
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            robot.read();
//            if(gamepad1.dpad_up){
//                // Not tuned
//                CommandScheduler.getInstance().schedule(
//               // new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_FROM_INTAKE)),
//                //new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SAMPLES)
//                ));
//            }
//            if(gamepad1.dpad_down){
//                CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
//                new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_RIGHT)),
//               // new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_TRANSFER))
//                ));
//            }
//            if(gamepad1.dpad_left){
//                CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
//                new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT)),
//               // new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_TRANSFER))));
//            }
//
//            if(gamepad1.cross){ // intake -> outtake transfer.
//                /*
//                Pseudocode: move slides to appropriate position, rotate outtake arm  and rotate outtake claw, close outtake claw and open intake claw, move arm.
//                Optimize with ParallelCommandGroup when necessary
//                 */
//                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                        new ParallelCommandGroup(new InstantCommand(() -> robot.intake.setClaw(IntakeSubsystem.CLAW_CLOSE)),
////                        new LiftPositionCommand(robot.h_lift, 20, 4, 20, 2)), new WaitCommand(100),
//                        new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_FROM_INTAKE)), new WaitCommand(100),
//            //            new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_TRANSFER)),
//                       // new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN)), new WaitCommand(100),
//                      //  new InstantCommand(() -> robot.intake.setClaw(IntakeSubsystem.CLAW_OPEN)), new WaitCommand(100)
////                        new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_RIGHT)),
////                        new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SAMPLES))
//                ));
//            }
//
//            if(gamepad1.square){ // specimen intake
//                // Move intake claw to correct rotation, shift horizontal slide into submersible, rotate intake claw and pick up sample and retract to prepare for outtake transfer.
//                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                        new InstantCommand(() -> robot.intake.setRotate(IntakeSubsystem.ROTATE_ENTER)), new WaitCommand(100),
//                       // new LiftPositionCommand(robot.h_lift, 30, 4, 20, 2), new WaitCommand(100),
//                      //  new InstantCommand(() -> robot.intake.setRotate(IntakeSubsystem.ROTATE_PICKUP)), new WaitCommand(300),
//                      //  new InstantCommand(() -> robot.intake.setClaw(IntakeSubsystem.CLAW_CLOSE)),
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> robot.intake.setRotate(IntakeSubsystem.ROTATE_ENTER)),
//                         //       new LiftPositionCommand(robot.h_lift, 20, 4, 20, 2)
//                        )
//                ));
//            }
//            if(gamepad1.triangle){
//                // move vertical slides to appropriate height, rotate outtake arm / claw to appropriate position to clip sample???
//                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                        new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT)),  new WaitCommand(100), // roughly this arm pos?
//                      //  new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN)),  new WaitCommand(100),
//                       // new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN)),
//                        new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_RIGHT))
//                ));
//            }
//            if(gamepad1.circle){
//                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
////                        new LiftPositionCommand2(robot.v_lift, 20, 2, 20, 2),
////                        new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.A)),  new WaitCommand(100), // roughly this arm pos?
//                      //  new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_DEPOSIT)),  new WaitCommand(100),
//                      //  new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN)),
//                      //  new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_TRANSFER)), // we don't want claw to get caught in the bin when we retract.
//                        //new LiftPositionCommand2(robot.v_lift, -2, 2, 20, 2),
//                        new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_FROM_INTAKE))
//                ));
//            }
//
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//            double botHeading = Math.toRadians(90) + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
//
//            CommandScheduler.getInstance().run();
//            robot.write();
//            robot.loop();
//        }
//    }
//}
