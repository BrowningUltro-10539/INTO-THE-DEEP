package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class IntakeSubsystem{
    private ServoImplEx leftServo, rightServo, rotate, claw;
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
        leftServo = hardwareMap.get(ServoImplEx.class, "leftPivot");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightPivot");
        rotate = hardwareMap.get(ServoImplEx.class, "rotate");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
    }

    public void update(ClawState state){
        switch(state){
            case CLOSED:
                setClaw(CLAW_OPEN);
                break;
            case OPEN:
                setClaw(CLAW_CLOSE);
                break;
        }
    }

    public void update(RotateState state){
        switch (state){
            case INTAKE:
                setRotate(ROTATE_INTAKE);
                break;
            case TRANSFER:
                setRotate(ROTATE_OUTTAKE);
        }
    }

    public void setClaw(double pos){claw.setPosition(pos);}
    public void setArm(double pos){
        leftServo.setPosition(pos);
        rightServo.setPosition(1 - pos);
    }
    public void setRotate(double pos){rotate.setPosition(pos);}
    public void read(){}
    public void write(){}
    public void loop(){}
}
