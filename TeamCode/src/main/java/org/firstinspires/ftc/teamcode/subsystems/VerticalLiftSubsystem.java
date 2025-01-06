package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VerticalLiftSubsystem {
    public final MotorEx lift1, lift2;
    private MotionProfile profile;
    public MotionState currentState;

    private final ElapsedTime timer, voltageTimer;
    private final PIDController liftController;
    private final VoltageSensor voltageSensor;


    private double voltage;
    private double liftPosition;


    public static double P = 0.26;
    public static  double I = 0;
    public static double D = 0;
    public static double kG = 0.21;

    private final double SLIDE_TICKS_PER_INCH = 2 * Math.PI * 0.764445002 / 145.1;

    private boolean isAuto = false;

    public double liftPower = 0.0;
    public double liftTargetPosition = 0.0;
    public VerticalLiftSubsystem(HardwareMap hardwareMap, boolean isAuto){
        lift1 = new MotorEx(hardwareMap, "verticalMotor1");
        lift2 = new MotorEx(hardwareMap, "verticalMotor2");

        lift2.setInverted(true);
        lift2.encoder.setDirection(MotorEx.Direction.REVERSE);

    }
}
