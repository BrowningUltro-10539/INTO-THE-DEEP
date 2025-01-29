package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public MecanumDrive driveSubsystem;
    public HangerSubsystem test;

    public IntakeSubsystem intake;
    public OuttakeSubsystem outtake;
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
        v_lift = new VerticalLiftSubsystem(hardwareMap, isAuto);
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
        h_lift.read();
        v_lift.read();
        if(isAuto){
            //driveSubsystem.getPoseEstimate();
        }
    }

    public void write(){
        intake.write();
        v_lift.write(); h_lift.write();
        if(isAuto){
            //driveSubsystem.update();
        }
    }

    public void loop(){
        h_lift.loop();
        v_lift.loop();
    }

    public void reset(){
        h_lift.horizontalLift.resetEncoder();
        v_lift.lift2.resetEncoder();
        h_lift.horizontalLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        v_lift.lift1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        v_lift.lift2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setRotate(IntakeSubsystem.ROTATE_INTAKE);
        intake.setClaw(IntakeSubsystem.CLAW_CLOSE);
    }

    public List<LynxModule> getControllers(){
        return controllers;
    }

    public Pose2d getPose(){
        return robotPose;
    }


}
