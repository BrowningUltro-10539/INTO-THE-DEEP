package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.SampleMecanumDrive;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 200, Math.toRadians(360), Math.toRadians(360), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-13, -60, Math.PI/2))
                        .lineTo(new Vector2d(0, -32))
                        .waitSeconds(0.5)
                        .forward(-6)
                        .strafeRight(30)
                        .lineTo(new Vector2d(46.7, -9.2)) // travel slightly above first sample
                        .forward(-50) // first sample
                        .forward(50)
                        .strafeRight(10)
                        .forward(-50)
                        .forward(50) // asdf
                        .strafeRight(5)
                        .forward(-45)
                        .strafeLeft(15)
                        .turn(Math.PI)
                        .waitSeconds(0.5)
                        .turn(Math.PI)
                        .lineTo(new Vector2d(0, -32))
                        .waitSeconds(0.5)
                        .lineTo(new Vector2d(46.7, -54.2))
                        .turn(Math.PI)
                        .waitSeconds(0.5)
                        .turn(Math.PI)
                        .lineTo(new Vector2d(0, -32))
                        .waitSeconds(0.5)
                        .lineTo(new Vector2d(46.7, -54.2))
                        .turn(Math.PI)
                        .waitSeconds(0.5)
                        .turn(Math.PI)
                        .lineTo(new Vector2d(0, -32))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}