package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class OuttakeSubsystem extends SubsystemBase {
    public Servo leftArm, rightArm, claw, rotate;
    public static double ARM_RIGHT = 0;
    public static double ARM_PICKUP_FROM_INTAKE = 0.8; // position of outtake arm that picks up from intake claw
    public static double ARM_MIDPOINT = 0.6; // arm mid point

    public static double ARM_DEPOSIT = 0.85; // arm position when we deposit samples and specimen.
    public static double ARM_PICKUP_SPECIMEN = 0.38; // arm position when we intake specimen.
    public static double CLAW_OPEN = 0.5, CLAW_CLOSE = 0.32, claw_pos = 0.32;

    public static double ROTATE_SPECIMEN_PICKUP = 0.18; // dpad up (outtake claw faces wall to grab specimen hanging on the wall)
    public static double ROTATE_SPECIMEN_SCORE = 0.6; // outtake claw faces

    public OuttakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        leftArm = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightArm = hardwareMap.get(ServoImplEx.class, "rightArm");
        claw = hardwareMap.get(ServoImplEx.class, "outtakeClawServo");
        rotate = hardwareMap.get(ServoImplEx.class, "outtakeRotateServo");
    }

    public void loop(){}
    public void read(){}
    public void write(){}
    public void setRotate(double pos){
        rotate.setPosition(pos);
    }
    public void setClaw(double pos){
        claw.setPosition(pos);
    }


    public void setArmPos(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }
}
