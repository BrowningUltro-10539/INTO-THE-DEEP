package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class OuttakeSubsystem {
    public ServoImplEx leftArm, rightArm, claw, rotate;
    public static double ARM_RIGHT = 0;
    public static double ARM_PICKUP = 0.8; // position of outtake arm that picks up from intake claw
    public static double ARM_MIDPOINT = 0.4;

    public static double ARM_DEPOSIT = 0.2;// max height


    public static double CLAW_OPEN = 0.5, CLAW_CLOSE = 0.32;

    public static double ROTATE_TRANSFER = 0; // claw rotate position to transfer from intake -> outtake
    public static double ROTATE_SPECIMEN = 0; // claw rotate position to place specimen.
    public static double ROTATE_SAMPLES = 0;

    public OuttakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        leftArm = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightArm = hardwareMap.get(ServoImplEx.class, "rightArm");
        claw = hardwareMap.get(ServoImplEx.class, "outtakeClawServo");
        rotate = hardwareMap.get(ServoImplEx.class, "outtakeRotateServo");
    }

    public void loop(){}
    public void read(){}
    public void write(){}
    public void setRotate(double pos){rotate.setPosition(pos);}
    public void setClaw(double pos){claw.setPosition(pos);}

    public void setArmPos(double pos){
        leftArm.setPosition(pos);
        rightArm.setPosition(1 - pos);
    }
}
