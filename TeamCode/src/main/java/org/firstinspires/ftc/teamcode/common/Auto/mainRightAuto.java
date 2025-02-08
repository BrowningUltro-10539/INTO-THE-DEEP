package org.firstinspires.ftc.teamcode.common.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class mainRightAuto extends LinearOpMode {
    private Robot robot;



    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        this.robot = new Robot(hardwareMap, true);
        
        //start
        Pose2d startPose = new Pose2d(10, -60, Math.PI/2);
        robot.drive.setPoseEstimate(startPose);
        robot.reset();

        //to depo
        TrajectorySequence toDepo = robot.drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(0, -32))
                .build();

        //to park
        TrajectorySequence toSamplePush1 = robot.drive.trajectorySequenceBuilder(toDepo.end())
                .splineToConstantHeading(new Vector2d(46.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample1 = robot.drive.trajectorySequenceBuilder(toSamplePush1.end())
                .forward(-50)
                .build();

        TrajectorySequence toSamplePush2 = robot.drive.trajectorySequenceBuilder(toObservationSample1.end())
                .forward(40)
                .splineToConstantHeading(new Vector2d(56.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample2 = robot.drive.trajectorySequenceBuilder(toSamplePush2.end())
                .forward(-50)
                .build();

        TrajectorySequence toSamplePush3 = robot.drive.trajectorySequenceBuilder(toObservationSample2.end())
                .forward(40)
                .splineToConstantHeading(new Vector2d(61.7, -9.2), 0)
                .build();

        TrajectorySequence toObservationSample3 = robot.drive.trajectorySequenceBuilder(toSamplePush3.end())
                .forward(-45)
                .strafeLeft(15)
                .build();

        TrajectorySequence turnToPickup = robot.drive.trajectorySequenceBuilder(toObservationSample3.end())
                .turn(Math.PI)
                .build();

        TrajectorySequence turnAndDepo1 = robot.drive.trajectorySequenceBuilder(turnToPickup.end())
                .turn(-Math.PI)
                .lineTo(new Vector2d(0, -32))
                .build();


        TrajectorySequence toPickupAndTurn1 = robot.drive.trajectorySequenceBuilder(turnAndDepo1.end())
                .lineTo(new Vector2d(46.7, -54.2))
                .turn(Math.PI)
                .build();

        TrajectorySequence turnAndDepo2 = robot.drive.trajectorySequenceBuilder(toPickupAndTurn1.end())
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



        telemetry.setMsTransmissionInterval(50);

        while(!isStarted() && !isStopRequested()) {
            robot.reset();

            for (LynxModule module : robot.getControllers()) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }

            while (opModeIsActive()) {
                robot.read();
                //depo1

                CommandScheduler.getInstance().schedule(new TrajectorySequenceFollowerCommand(robot.drive, toDepo));
//                robot.drive.followTrajectorySequence(toDepo);
//                //push samples
//                robot.drive.followTrajectorySequence(toSamplePush1);
//                robot.drive.followTrajectorySequence(toObservationSample1);
//                robot.drive.followTrajectorySequence(toSamplePush2);
//                robot.drive.followTrajectorySequence(toObservationSample2);
//                robot.drive.followTrajectorySequence(toSamplePush3);
//                robot.drive.followTrajectorySequence(toObservationSample3);
//                robot.drive.followTrajectorySequence(turnToPickup);
//                //depo 2
//                robot.drive.followTrajectorySequence(turnAndDepo1);
//                robot.drive.followTrajectorySequence(toPickupAndTurn1);
//                //depo 3
//                robot.drive.followTrajectorySequence(turnAndDepo2);
//                robot.drive.followTrajectorySequence(toPickupAndTurn2);
//                //depo 4
//                robot.drive.followTrajectorySequence(turnAndDepo3);
//                robot.drive.followTrajectorySequence(toPickupAndTurn3);


                CommandScheduler.getInstance().run();


                robot.write();
                robot.drive.update();

                telemetry.addData("Robot Pose: ", robot.drive.getLocalizer().getPoseEstimate());
                telemetry.update();

                for (LynxModule module : robot.getControllers()) {
                    module.clearBulkCache();
                }

            }


        }

}

