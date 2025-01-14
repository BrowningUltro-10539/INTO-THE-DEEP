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

public class HorizontalLiftSubsystem extends SubsystemBase {
    public final MotorEx horizontalLift;
    private final PIDController controller;
    private MotionProfile profile;
    public MotionState state;
    private final VoltageSensor voltageSensor;

    public static double P = 0.17, I = 0.0, D = 0.0;
    private double voltage;
    private final ElapsedTime timer, voltageTimer;
    private double liftPosition, targetLiftPosition;

    public double liftPower;
    private final double SLIDE_TICK = 2 * Math.PI * 0.701771654 / 145.1;
    private boolean isAuto = false;


    public HorizontalLiftSubsystem(HardwareMap hardwareMap, boolean isAuto){
        horizontalLift = new MotorEx(hardwareMap, "horizontalMotor");
        controller = new PIDController(P, I, D);
        controller.setPID(P, I, D);
        this.isAuto = isAuto;


        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0),
                new MotionState(0,0), 0, 0);
        // values are not final
        timer = new ElapsedTime(); voltageTimer = new ElapsedTime();
        voltageSensor = hardwareMap.get(VoltageSensor.class, "voltageSensor");
    }

    public void loop(){
        // ask washieu about how on earth voltage plays a role in the loop :skull:
        // obviously I don't want to take anything for granted.
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        state = profile.get(timer.time());
        if (state.getV() != 0) {
            targetLiftPosition = state.getX();
        }
        liftPower = controller.calculate(liftPosition, targetLiftPosition) / voltage * 14;
    }
    public void incrementSlide(double increment){
        if(state.getV() != 0){
            targetLiftPosition = liftPosition + increment;
        }
    }
    public void read(){
        liftPosition = horizontalLift.encoder.getPosition() * SLIDE_TICK;
    }
    public void write(){
        horizontalLift.set(liftPower);
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
