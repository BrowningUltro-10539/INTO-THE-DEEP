package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
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

    public MecanumDrive drive;
    public HangerSubsystem test;
    public SampleMecanumDrive sampleDrive;

    public IntakeSubsystem intake;
    public OuttakeSubsystem outtake;
    public HorizontalLiftSubsystem h_lift;
    public VerticalLiftSubsystem v_lift;

//    public HangerSubsystem hanger;

    public HardwareMap hardwareMap;

    private boolean isAuto = false;
    public Pose2d robotPose;
    private List<LynxModule> controllers;

    public Robot(HardwareMap hardwareMap, boolean isAuto){
        this.isAuto = isAuto;
        this.hardwareMap = hardwareMap;
        drive = new MecanumDrive(sampleDrive, isAuto);
        this. sampleDrive = new SampleMecanumDrive(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, isAuto);
        h_lift = new HorizontalLiftSubsystem(hardwareMap, isAuto);
        v_lift = new VerticalLiftSubsystem(hardwareMap, isAuto);
//        if(isAuto){
////            lift.rightArm.encoder.reset();
//        }
        outtake = new OuttakeSubsystem(hardwareMap, isAuto);
        controllers = hardwareMap.getAll(LynxModule.class);
    }

    public Robot(HardwareMap hardwareMap){
        this(hardwareMap, false);
    }

    public void read(){
        intake.read();
        h_lift.read();
        v_lift.read();
        if(isAuto){
            //driveSubsystem.getPoseEstimate();
        }
    }

    public void write(){
        intake.write();
        v_lift.write();
        h_lift.write();
        outtake.write();
        //careful here
//        drive.update();
    }

    public void loop(){
        h_lift.loop();
        v_lift.loop();
    }

    public void reset(){
        intake.setRotate(IntakeSubsystem.ROTATE_DOWN);
        intake.setClaw(IntakeSubsystem.CLAW_CLOSE);
        outtake.setClaw(OuttakeSubsystem.CLAW_CLOSE);
        outtake.setRotate(OuttakeSubsystem.ROTATE_INIT);
        outtake.setArmPos(OuttakeSubsystem.ARM_PICKUP_SPECIMEN);
    }

    public List<LynxModule> getControllers(){
        return controllers;
    }

    public Pose2d getPose(){
        return robotPose;
    }


}
