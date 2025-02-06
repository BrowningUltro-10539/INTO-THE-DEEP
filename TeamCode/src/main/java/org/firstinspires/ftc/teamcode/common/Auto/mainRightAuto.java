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
        robot.sampleDrive.setPoseEstimate(startPose);
        robot.reset();

        //to depo
        TrajectorySequence toDepo = robot.sampleDrive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(0, -32))
                .build();

        //to park
        TrajectorySequence toSamplePush1 = robot.sampleDrive.trajectorySequenceBuilder(toDepo.end())
                .splineToConstantHeading(new Vector2d(46.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample1 = robot.sampleDrive.trajectorySequenceBuilder(toSamplePush1.end())
                .forward(-50)
                .build();

        TrajectorySequence toSamplePush2 = robot.sampleDrive.trajectorySequenceBuilder(toObservationSample1.end())
                .forward(40)
                .splineToConstantHeading(new Vector2d(56.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample2 = robot.sampleDrive.trajectorySequenceBuilder(toSamplePush2.end())
                .forward(-50)
                .build();

        TrajectorySequence toSamplePush3 = robot.sampleDrive.trajectorySequenceBuilder(toObservationSample2.end())
                .forward(40)
                .splineToConstantHeading(new Vector2d(61.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample3 = robot.sampleDrive.trajectorySequenceBuilder(toSamplePush3.end())
                .forward(-45)
                .strafeLeft(15)
                .build();

        TrajectorySequence turnToPickup = robot.sampleDrive.trajectorySequenceBuilder(toObservationSample3.end())
                .turn(Math.PI)
                .build();

        TrajectorySequence turnAndDepo1 = robot.sampleDrive.trajectorySequenceBuilder(turnToPickup.end())
                .turn(-Math.PI)
                .lineTo(new Vector2d(0, -32))
                .build();


        TrajectorySequence toPickupAndTurn1 = robot.sampleDrive.trajectorySequenceBuilder(turnAndDepo1.end())
                .lineTo(new Vector2d(46.7, -54.2))
                .turn(Math.PI)
                .build();

        TrajectorySequence turnAndDepo2 = robot.sampleDrive.trajectorySequenceBuilder(toPickupAndTurn1.end())
                .turn(-Math.PI)
                .lineTo(new Vector2d(0, -32))
                .build();

        TrajectorySequence toPickupAndTurn2 = robot.sampleDrive.trajectorySequenceBuilder(turnAndDepo2.end())
                .lineTo(new Vector2d(46.7, -54.2))
                .turn(Math.PI)
                .build();

        TrajectorySequence turnAndDepo3 = robot.sampleDrive.trajectorySequenceBuilder(toPickupAndTurn2.end())
                .turn(-Math.PI)
                .lineTo(new Vector2d(0, -32))
                .build();

        TrajectorySequence toPickupAndTurn3 = robot.sampleDrive.trajectorySequenceBuilder(turnAndDepo3.end())
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
                robot.sampleDrive.followTrajectorySequence(toDepo);
                sleep(500);
                //push samples
                robot.drive.followTrajectorySequence(toSamplePush1);
                robot.sampleDrive.followTrajectorySequence(toObservationSample1);
                robot.sampleDrive.followTrajectorySequence(toSamplePush2);
                robot.sampleDrive.followTrajectorySequence(toObservationSample2);
                robot.sampleDrive.followTrajectorySequence(toSamplePush3);
                robot.sampleDrive.followTrajectorySequence(toObservationSample3);
                robot.sampleDrive.followTrajectorySequence(turnToPickup);
                //depo 2
                robot.sampleDrive.followTrajectorySequence(turnAndDepo1);
                robot.sampleDrive.followTrajectorySequence(toPickupAndTurn1);
                //depo 3
                robot.sampleDrive.followTrajectorySequence(turnAndDepo2);
                robot.sampleDrive.followTrajectorySequence(toPickupAndTurn2);
                //depo 4
                robot.sampleDrive.followTrajectorySequence(turnAndDepo3);
                robot.sampleDrive.followTrajectorySequence(toPickupAndTurn3);


                CommandScheduler.getInstance().run();


                robot.write();
                robot.sampleDrive.update();

                telemetry.addData("Robot Pose: ", robot.sampleDrive.getLocalizer().getPoseEstimate());
                telemetry.update();

                for (LynxModule module : robot.getControllers()) {
                    module.clearBulkCache();
                }

            }


        }

    }
}
