package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizontalLiftSubsystem extends SubsystemBase {
    public final MotorEx motor;
    private final PIDController controller;
    private MotionProfile profile;
    public MotionState state;
    public static double P = 0.17;
    public static double I = 0;
    public static double D = 0;
    public double currentLiftPosition;
    public double targetLiftPosition;
    public double liftPower;
    private final double slidesTickPerInch = 2 * Math.PI * 0.701771654 / 145.1;
    private boolean isAuto = false;
    public HorizontalLiftSubsystem(HardwareMap hardwareMap, boolean isAuto){
        motor = new MotorEx(hardwareMap, "horizontalMotor");
        controller = new PIDController(P, I, D);
        controller.setPID(P, I, D);
        this.isAuto = isAuto;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0),
                new MotionState(0,0), 0, 0);
        // values are not final
    }
}
