package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalLiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemDouble;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;

import java.util.List;

public class Robot {

    public SampleMecanumDrive drive; // Use SampleMecanumDrive directly
    public HangerSubsystem test;

    public IntakeSubsystem intake;
    public OuttakeSubsystem outtake;
    public HorizontalLiftSubsystem h_lift;
    public VerticalLiftSubsystem v_lift;

    public HardwareMap hardwareMap;

    private boolean isAuto = false;
    public Pose2d robotPose;
    private List<LynxModule> controllers;

    public Robot(HardwareMap hardwareMap, boolean isAuto) {
        this.isAuto = isAuto;
        this.hardwareMap = hardwareMap;

        // Initialize SampleMecanumDrive (RoadRunner drive)
        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize other subsystems
        intake = new IntakeSubsystem(hardwareMap, isAuto);
        h_lift = new HorizontalLiftSubsystem(hardwareMap, isAuto);
        v_lift = new VerticalLiftSubsystem(hardwareMap, isAuto);
        outtake = new OuttakeSubsystem(hardwareMap, isAuto);

        if(isAuto){
            v_lift.lift2.encoder.reset();
            h_lift.horizontalLift.resetEncoder();
        }

        controllers = hardwareMap.getAll(LynxModule.class);
    }

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public void read() {
        outtake.read();
        intake.read();
        h_lift.read();
        v_lift.read();

        if(isAuto){
            robotPose = drive.getPoseEstimate();
        }

    }

    public void write() {
        intake.write();
        v_lift.write();
        h_lift.write();
        outtake.write();
        if(isAuto){
            drive.update();
        }
    }

    public void loop() {
        h_lift.loop();
        v_lift.loop();
    }

    public void reset() {
        h_lift.horizontalLift.resetEncoder();
        v_lift.lift2.resetEncoder();
    }

    public List<LynxModule> getControllers() {
        return controllers;
    }

    public Pose2d getPose() {
        return robotPose;
    }
}
