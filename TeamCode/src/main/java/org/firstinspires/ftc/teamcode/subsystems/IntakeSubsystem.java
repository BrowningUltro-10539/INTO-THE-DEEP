package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class IntakeSubsystem{
    public Servo clawServo, rotate;
    // private final VoltageSensor voltageSensor;

    public static double CLAW_OPEN = 0.5;
    public static double CLAW_CLOSE = 0.32;

    public static double ROTATE_INTAKE = 0.275;
    public static double ROTATE_ENTER = 0.55; // position to enter submersible
    public static double ROTATE_TRANSFER = 1;
    public static double ROTATE_PICKUP = 0;

    public enum IntakeState {INTAKE_OUTTAKE_TRANSFER}
    private double voltage;

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto){
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        rotate = hardwareMap.get(Servo.class, "rotate");
    }

    public void setClaw(double pos){clawServo.setPosition(pos);}
    public void setRotate(double pos){rotate.setPosition(pos);}
    public void read(){}
    public void write(){}
    public void loop(){}
}
