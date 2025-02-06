package org.firstinspires.ftc.teamcode.common.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class mainRightAuto extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        this.robot = new Robot(hardwareMap, true);

        //start
        Pose2d startPose = new Pose2d(10, -60, Math.PI/2);
        robot.driveSubsystem.setPoseEstimate(startPose);
        robot.reset();

        //to depo
        TrajectorySequence toDepo = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(0, -32))
                .build();

        //to park
        TrajectorySequence toSamplePush1 = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(46.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample1 = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .forward(-50)
                .build();

        TrajectorySequence toSamplePush2 = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .forward(40)
                .splineToConstantHeading(new Vector2d(56.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample2 = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .forward(-50)
                .build();

        TrajectorySequence toSamplePush3 = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .forward(40)
                .splineToConstantHeading(new Vector2d(61.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample3 = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .forward(-45)
                .strafeLeft(15)
                .build();

        TrajectorySequence turnToPickup = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .turn(Math.PI)
                .build();

        TrajectorySequence turnAndDepo = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .turn(-Math.PI)
                .lineTo(new Vector2d(0, -32))
                .build();

        TrajectorySequence toPickupAndTurn = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(46.7, -54.2))
                .turn(Math.PI)
                .build();



        telemetry.setMsTransmissionInterval(50);

        while(!isStarted() && !isStopRequested()){
            robot.reset();

            for(LynxModule module : robot.getControllers()){
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            robot.reset();

            while (opModeIsActive()) {
                robot.read();
                //depo1
                robot.driveSubsystem.followTrajectorySequence(toDepo);
                //push samples
                robot.driveSubsystem.followTrajectorySequence(toSamplePush1);
                robot.driveSubsystem.followTrajectorySequence(toObservationSample1);
                robot.driveSubsystem.followTrajectorySequence(toSamplePush2);
                robot.driveSubsystem.followTrajectorySequence(toObservationSample2);
                robot.driveSubsystem.followTrajectorySequence(toSamplePush3);
                robot.driveSubsystem.followTrajectorySequence(toObservationSample3);
                robot.driveSubsystem.followTrajectorySequence(turnToPickup);
                //depo 2
                robot.driveSubsystem.followTrajectorySequence(turnAndDepo);
                robot.driveSubsystem.followTrajectorySequence(toPickupAndTurn);
                //depo 3
                robot.driveSubsystem.followTrajectorySequence(turnAndDepo);
                robot.driveSubsystem.followTrajectorySequence(toPickupAndTurn);
                //depo 4
                robot.driveSubsystem.followTrajectorySequence(turnAndDepo);
                robot.driveSubsystem.followTrajectorySequence(toPickupAndTurn);


                CommandScheduler.getInstance().run();


                robot.write();

                telemetry.addData("Robot Pose: ", robot.driveSubsystem.getLocalizer().getPoseEstimate());
                telemetry.update();

                for (LynxModule module : robot.getControllers()) {
                    module.clearBulkCache();
                }

            }


        }

    }
}
