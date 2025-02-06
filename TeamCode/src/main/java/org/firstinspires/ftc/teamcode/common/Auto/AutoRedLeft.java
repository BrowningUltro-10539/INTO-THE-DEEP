package org.firstinspires.ftc.teamcode.common.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutoRedLeft extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap, false);
        robot.reset();
        Pose2d startPos = new Pose2d(-13, -60, Math.PI/2);
        robot.drive.setPoseEstimate(startPos);

        // path to first specimen deposit
        Trajectory toSpecimenScore = robot.drive.trajectoryBuilder(startPos)
                .lineTo(new Vector2d(0, -32))
                .build();

        // deposit samples and path to pick up first specimen
        TrajectorySequence depositSamples = robot.drive.trajectorySequenceBuilder(toSpecimenScore.end())
                .lineTo(new Vector2d(27.5, -34.11))
                .splineToConstantHeading(new Vector2d(46.7, -9.2), 0)
                .forward(-50)
                .forward(40)
                .splineToConstantHeading(new Vector2d(56.7, -9.2), 0)
                .forward(-50)
                .forward(40)
                .splineToConstantHeading(new Vector2d(61.7, -9.2), 0)
                .forward(-45)
                .strafeLeft(15)
                .turn(Math.PI/2)
                .build();

        // path to deposit remaining specimens
        TrajectorySequence depositSpecimen = robot.drive.trajectorySequenceBuilder(new Pose2d(46.7, -54.2, Math.PI / 2))
                .turn(-Math.PI)
                .lineTo(new Vector2d(0, -32))
                .build();

        // path to pick up remaining specimens.
        TrajectorySequence pickupSpecimen = robot.drive.trajectorySequenceBuilder(new Pose2d(0, -32, Math.PI / 2))
                .lineTo(new Vector2d(46.7, -54.2))
                .turn(Math.PI)
                .build();

        waitForStart();
        if(isStopRequested()) return;
        // prepare for deposit
        robot.intake.setRotate(IntakeSubsystem.ROTATE_ENTER);
        robot.outtake.setClaw(0.45);
        robot.outtake.setArmPos(OuttakeSubsystem.ARM_MIDPOINT);
        robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP);

        // deposit first specimen
        robot.drive.followTrajectory(toSpecimenScore);
        robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT);
        sleep(1250);
        robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN);

        // move samples to observation zone and prepare to pick up first specimen.
        robot.drive.followTrajectorySequence(depositSamples);

        for(int i = 0; i < 3; i++){
            // pick up specimens
            robot.outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_SPECIMEN);
            robot.outtake.setRotate(OuttakeSubsystem.ROTATE_SPECIMEN_PICKUP);
            robot.outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE);
            robot.outtake.setClaw(OuttakeSubsystem.ARM_MIDPOINT);
            // go to high chamber
            robot.drive.followTrajectorySequence(depositSpecimen);
            // score
            robot.outtake.setArmPos(OuttakeSubsystem.ARM_DEPOSIT);
            sleep(1250);
            robot.outtake.setClaw(OuttakeSubsystem.CLAW_OPEN);
            // go back and repeat
            robot.drive.followTrajectorySequence(pickupSpecimen);
        }
    }
}
