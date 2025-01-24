package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class IntakeSubsystem{
    public ServoImplEx clawServo, rotate;
    // private final VoltageSensor voltageSensor;

    public static double CLAW_OPEN = 0.7;
    public static double CLAW_CLOSE = 0.1;

    public static double ROTATE_INTAKE = 0.1;
    public static double ROTATE_OUTTAKE = 0.1;

    public static double ARM_START = 0;
    public static double ARM_INTAKE = 1;
    public static double ARM_THRU = 0;
    public static double ARM_DEPO = -1;
    public enum IntakeState { INTAKE, DECIDE, OPEN_CLAW, CLOSE_CLAW}
    public enum ArmState {ARM_START, ARM_INTAKE, ARM_THRU, ARM_DEPO}
    public enum ClawState { OPEN, CLOSED }
    public enum RotateState { INTAKE, TRANSFER }
    private double voltage;

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        clawServo = hardwareMap.get(ServoImplEx.class, "clawServo");
        rotate = hardwareMap.get(ServoImplEx.class, "rotate");
    }

    public void setClaw(double pos){clawServo.setPosition(pos);}
    public void setRotate(double pos){rotate.setPosition(pos);}
    public void read(){}
    public void write(){}
    public void loop(){}
}
