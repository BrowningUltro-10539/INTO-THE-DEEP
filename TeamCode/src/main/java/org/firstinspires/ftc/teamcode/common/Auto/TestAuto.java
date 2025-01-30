package org.firstinspires.ftc.teamcode.common.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand2;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@Autonomous
public class TestAuto extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        this.robot = new Robot(hardwareMap, true);

        //start
        Pose2d startPose = new Pose2d(33.0,60, Math.toRadians(90));
        robot.driveSubsystem.setPoseEstimate(startPose);
        robot.reset();

        // see diagrams in portfolio for better understanding of trajectories.
        Trajectory toScoreSpecimen = robot.driveSubsystem.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0,22), 0)
                .build();

        Trajectory firstSpecimen = robot.driveSubsystem.trajectoryBuilder(toScoreSpecimen.end())
                .splineTo(new Vector2d(-30, 38), 0)
                .splineTo(new Vector2d(-45, 0), 0) // prevents colliding with submersible.
                .build();

        Trajectory firstDeposit = robot.driveSubsystem.trajectoryBuilder(firstSpecimen.end())
                .splineTo(new Vector2d(-54,64), 0)
                .build();

        Trajectory secondSpecimen = robot.driveSubsystem.trajectoryBuilder(firstDeposit.end())
                .splineTo(new Vector2d(-60,15), 0)
                .build();

        Trajectory secondDeposit = robot.driveSubsystem.trajectoryBuilder(secondSpecimen.end())
                .splineTo(new Vector2d(-64,64), 0)
                .build();

        Trajectory thirdSpecimen = robot.driveSubsystem.trajectoryBuilder(secondDeposit.end())
                .splineTo(new Vector2d(-68,15), 0)
                .build();

        Trajectory thirdDeposit = robot.driveSubsystem.trajectoryBuilder(thirdSpecimen.end())
                .splineTo(new Vector2d(-70,64), 0)
                .build();

        Trajectory specimenPickup = robot.driveSubsystem.trajectoryBuilder(thirdDeposit.end())
                .splineTo(new Vector2d(-52,70), 0)
                .build();

        Trajectory specimenPickup2 = robot.driveSubsystem.trajectoryBuilder(toScoreSpecimen.end())
                .splineTo(new Vector2d(-52,70), 0)
                .build();

        boolean samples_collected = false;
        telemetry.setMsTransmissionInterval(50);

        while(isStarted() && !isStopRequested()){
            robot.reset();

            for(LynxModule module : robot.getControllers()){
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            robot.reset();

            while (opModeIsActive()) {
                robot.read();

                robot.driveSubsystem.followTrajectory(toScoreSpecimen);
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new LiftPositionCommand2(robot.v_lift, 15, 2, 20, 2),
                        new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT)),  new WaitCommand(100),
                        new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN)),  new WaitCommand(100),
                        new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN)),
                        new LiftPositionCommand2(robot.v_lift, -2, 2, 20, 2),
                        new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_RIGHT))
                ));
                if(!samples_collected){ // only do this when we haven't pushed the samples.
                    robot.driveSubsystem.followTrajectory(firstSpecimen);
                    robot.driveSubsystem.followTrajectory(firstDeposit);
                    robot.driveSubsystem.followTrajectory(secondSpecimen);
                    robot.driveSubsystem.followTrajectory(secondDeposit);
                    robot.driveSubsystem.followTrajectory(thirdSpecimen);
                    robot.driveSubsystem.followTrajectory(thirdDeposit);
                }
                robot.driveSubsystem.followTrajectory(samples_collected ? specimenPickup2 : specimenPickup);
                samples_collected = true;
                if(gamepad1.square){ // specimen intake
                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                            new InstantCommand(() -> robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_SPECIMEN)),
                            new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN)),
                            new InstantCommand(() -> robot.outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE)),
                            new InstantCommand(() -> robot.outtake.setRotate(OuttakeSubsystem.ROTATE_TRANSFER))
                    ));
                }
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
