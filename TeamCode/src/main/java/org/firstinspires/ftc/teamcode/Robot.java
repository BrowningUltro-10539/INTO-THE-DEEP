package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalLiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemDouble;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;

import java.util.List;

public class Robot {

    public MecanumDrive driveSubsystem;
    public HangerSubsystem test;

    public IntakeSubsystem intake;
    public HorizontalLiftSubsystem h_lift;
    public VerticalLiftSubsystem v_lift;
//    public MecanumDrive.HangerSubsystem hanger;

    private boolean isAuto = false;
    public Pose2d robotPose;
    private List<LynxModule> controllers;

    public Robot(HardwareMap hardwareMap, boolean isAuto){
        this.isAuto = isAuto;
//        driveSubsystem = new MecanumDrive(hardwareMap, robotPose);
        intake = new IntakeSubsystem(hardwareMap, isAuto);
        h_lift = new HorizontalLiftSubsystem(hardwareMap, isAuto);
//        hanger = new MecanumDrive.HangerSubsystem(hardwareMap, isAuto);

        if(isAuto){
//            lift.rightArm.encoder.reset();
        }

        controllers = hardwareMap.getAll(LynxModule.class);
    }

    public Robot(HardwareMap hardwareMap){
        this(hardwareMap, false);
    }

    public void read(){
        intake.read();
//        hanger.read();
//        lift.read();
        test.read();

        if(isAuto){
            //driveSubsystem.getPoseEstimate();
        }
    }

    public void write(){
        intake.write();
//        lift.write();

        if(isAuto){
            //driveSubsystem.update();
        }
    }

    public void reset(){
//        lift.rightArm.resetEncoder();
//        lift.rightArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        lift.leftArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        intake.update(IntakeSubsystem.ArmState.ARM_START);
//        intake.update(IntakeSubsystem.RotateState.INTAKE);
//        intake.update(IntakeSubsystem.ClawState.CLOSED);
    }

    public List<LynxModule> getControllers(){
        return controllers;
    }

    public Pose2d getPose(){
        return robotPose;
    }


}
