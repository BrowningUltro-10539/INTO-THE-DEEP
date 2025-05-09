package org.firstinspires.ftc.teamcode.common.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class fixedRightAuto extends LinearOpMode {
    private Robot robot;
    private double loopTime;

    private Pose2d startPose = new Pose2d(10, -60, Math.PI/2);
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, true);


        robot.drive.setPoseEstimate(startPose);
        robot.intake.setRotate(IntakeSubsystem.ROTATE_DOWN);
        robot.intake.setClaw(IntakeSubsystem.CLAW_CLOSE);
        robot.outtake.setRotate(OuttakeSubsystem.ROTATE_INIT);
        robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_SPECIMEN);
        sleep(1500);
        robot.outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE);



        //space for trajectories
        TrajectorySequence toDepoPreLoad = robot.drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(0, -25.8))
                .build();

        TrajectorySequence toDepoAdjust = robot.drive.trajectorySequenceBuilder(toDepoPreLoad.end())
                .lineTo(new Vector2d(0, -29.8))
                .build();

        TrajectorySequence toSamplePush1 = robot.drive.trajectorySequenceBuilder(toDepoAdjust.end())
                .lineTo(new Vector2d(36, -38.0))
                .lineTo(new Vector2d(36, -9))
                .splineToConstantHeading(new Vector2d(44.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample1 = robot.drive.trajectorySequenceBuilder(toSamplePush1.end())
                .forward(-45)
                .build();

        TrajectorySequence toSamplePush2 = robot.drive.trajectorySequenceBuilder(toObservationSample1.end())
                .forward(35)
                .splineToConstantHeading(new Vector2d(56.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample2 = robot.drive.trajectorySequenceBuilder(toSamplePush2.end())
                .forward(-45)
                .build();

        TrajectorySequence toSamplePush3 = robot.drive.trajectorySequenceBuilder(toObservationSample2.end())
                .forward(35)
                .splineToConstantHeading(new Vector2d(64, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample3 = robot.drive.trajectorySequenceBuilder(toSamplePush3.end())
                .forward(-40)
                .strafeLeft(15)
                .build();

        TrajectorySequence turnToPickup = robot.drive.trajectorySequenceBuilder(toObservationSample3.end())
                .turn(Math.toRadians(190))
                .build();

        TrajectorySequence forwardToGrab = robot.drive.trajectorySequenceBuilder(turnToPickup.end())
                .forward(2)
                .build();

        TrajectorySequence turnAndDepo1 = robot.drive.trajectorySequenceBuilder(forwardToGrab.end())
                .turn(-Math.toRadians(190))
                .lineTo(new Vector2d(5, -26))
                .build();

        TrajectorySequence turnAndDepo1PullBack = robot.drive.trajectorySequenceBuilder(turnAndDepo1.end())
                .lineTo(new Vector2d(5, -29.5))
                .build();


        TrajectorySequence toPark = robot.drive.trajectorySequenceBuilder(turnAndDepo1.end())
                .lineTo(new Vector2d(46.7, -60.2))
                .strafeRight(7)
                .forward(-5)
                .build();

        TrajectorySequence turnAndDepo2 = robot.drive.trajectorySequenceBuilder(toPark.end())
                .turn(-Math.PI)
                .lineTo(new Vector2d(0, -32))
                .build();

        TrajectorySequence toPickupAndTurn2 = robot.drive.trajectorySequenceBuilder(turnAndDepo2.end())
                .lineTo(new Vector2d(46.7, -54.2))
                .turn(Math.PI)
                .build();

        TrajectorySequence turnAndDepo3 = robot.drive.trajectorySequenceBuilder(toPickupAndTurn2.end())
                .turn(-Math.PI)
                .lineTo(new Vector2d(0, -32))
                .build();

        TrajectorySequence toPickupAndTurn3 = robot.drive.trajectorySequenceBuilder(turnAndDepo3.end())
                .lineTo(new Vector2d(46.7, -54.2))
                .turn(Math.PI)
                .build();


        while(!isStarted()){
            robot.read();
            robot.write();
        }


        //Auto Coded Here (so far prelaod only)
        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(new InstantCommand (() ->robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT)),
                        new InstantCommand(() -> robot.intake.setRotate(IntakeSubsystem.ROTATE_UP)),

                new SequentialCommandGroup(new InstantCommand(() -> robot.intake.setRotate(IntakeSubsystem.ROTATE_UP)),
                        new InstantCommand (() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT)),

                        new ParallelCommandGroup(
                                new InstantCommand(() ->robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP)),
                                new TrajectorySequenceFollowerCommand(robot.drive, toDepoPreLoad)
                        ),
                        new WaitCommand(100),
                        new SequentialCommandGroup(new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT)), new WaitCommand(300), new TrajectorySequenceFollowerCommand(robot.drive, toDepoAdjust), new WaitCommand(300), new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN)), new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT))),
                        new WaitCommand(100),
                        new TrajectorySequenceFollowerCommand(robot.drive, toSamplePush1), new TrajectorySequenceFollowerCommand(robot.drive, toObservationSample1),
                        new TrajectorySequenceFollowerCommand(robot.drive, toSamplePush2), new TrajectorySequenceFollowerCommand(robot.drive, toObservationSample2),
                        new TrajectorySequenceFollowerCommand(robot.drive, toSamplePush3), new TrajectorySequenceFollowerCommand(robot.drive, toObservationSample3),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_SPECIMEN)),
                                new InstantCommand(() ->robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_SCORE))
                        ),
                        new TrajectorySequenceFollowerCommand(robot.drive, turnToPickup),
                        new TrajectorySequenceFollowerCommand(robot.drive, forwardToGrab),
                        new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE)),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT)),
                        new WaitCommand(500),
                        new ParallelCommandGroup( new TrajectorySequenceFollowerCommand(robot.drive, turnAndDepo1), new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP))),
                        new SequentialCommandGroup(new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT)), new WaitCommand(300), new TrajectorySequenceFollowerCommand(robot.drive, turnAndDepo1PullBack), new WaitCommand(300), new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN)), new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT))),
                        new ParallelCommandGroup(new TrajectorySequenceFollowerCommand(robot.drive, toPark), new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT)))

                )
        ));

        waitForStart();

        robot.reset();

        while(opModeIsActive()){
            robot.read();

            CommandScheduler.getInstance().run();

            robot.intake.loop();
            robot.v_lift.loop();
            robot.h_lift.loop();
            robot.outtake.loop();

            robot.write();

            telemetry.addData("Current Pose: ", robot.drive.getLocalizer().getPoseEstimate());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();


        }
    }
}
