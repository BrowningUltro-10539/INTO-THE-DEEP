package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VerticalLiftSubsystem extends SubsystemBase {
    public final MotorEx lift1, lift2;
    private final PIDController controller;
    private MotionProfile profile;
    public MotionState state;

    public static double P = 0.15, I = 0.0, D = 0.0, kG = 0.027;
//    private final ElapsedTime timer, voltageTimer;
    private double liftPosition, targetLiftPosition;
    public VoltageSensor voltageSensor;

    public double liftPower, voltage;
    private final double SLIDE_TICK = 2 * Math.PI * 0.701771654 / 537.7;
    private boolean isAuto = false;
    public VerticalLiftSubsystem(HardwareMap hardwareMap, boolean isAuto){
        lift1 = new MotorEx(hardwareMap, "verticalM1");
        lift2 = new MotorEx(hardwareMap, "verticalM2");
        controller = new PIDController(P, I, D);
        controller.setPID(P, I, D);
        this.isAuto = isAuto;
    }

    public void loop(){
        liftPower = controller.calculate(liftPosition, targetLiftPosition) + kG;
    }

    public void read(){
        liftPosition = lift2.encoder.getPosition() * SLIDE_TICK;
    }

    public void write(){
        lift1.set(liftPower);
        lift2.set(liftPower);
    }

    public void setTargetLiftPosition(double v){targetLiftPosition = v;}

    public double getLiftPosition(){return liftPosition;}

    public void newProfile(double targetPos, double max_v, double max_a){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(liftPosition, 0),
                new MotionState(targetPos, 0), max_v, max_a
        );
    }

}
