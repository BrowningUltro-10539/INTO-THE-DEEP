package org.firstinspires.ftc.teamcode.common.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class BlueLeftAuto extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        this.robot = new Robot(hardwareMap, true);

        //start
        Pose2d startPose = new Pose2d(33.0,60, Math.toRadians(90));
        robot.driveSubsystem.setPoseEstimate(startPose);
        robot.reset();

        //to pickup
        Trajectory toPickup = robot.driveSubsystem.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(50,30), Math.toRadians(0))
                .build();

        //to score
        Trajectory toScore1 = robot.driveSubsystem.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(50,40), Math.toRadians(0))
                .build();

        Trajectory toScore2 = robot.driveSubsystem.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(60, 57), Math.toRadians(-135))
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

                robot.driveSubsystem.followTrajectory(toPickup);
                robot.driveSubsystem.followTrajectory(toScore1);
                robot.driveSubsystem.followTrajectory(toScore2);


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
