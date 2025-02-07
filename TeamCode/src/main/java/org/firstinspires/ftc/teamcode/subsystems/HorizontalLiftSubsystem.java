package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HorizontalLiftSubsystem extends SubsystemBase {


    public final DcMotorEx horizontalLift;
    private final PIDController controller;
    private MotionProfile profile;
    public MotionState state;
//    private final VoltageSensor voltageSensor;


    public static double P = 0.3, I = 0.0, D = 0.0;
    private double voltage;
//    private final ElapsedTime timer, voltageTimer;
    private double liftPosition, targetLiftPosition;

    public double liftPower = 0;
    private final double SLIDE_TICK = 2 * Math.PI * 0.701771654 / 384.5;
    private boolean isAuto = false;

    //-2, all the way back
    //14 full extension
    //4 for transfer when from full extension
    public HorizontalLiftSubsystem(HardwareMap hardwareMap, boolean isAuto){
        horizontalLift = hardwareMap.get(DcMotorEx.class, "horizontalMotor");
//        horizontalLift.encoder.setDirection(Motor.Direction.REVERSE);
        controller = new PIDController(P, I, D);
        controller.setPID(P, I, D);
        this.isAuto = isAuto;
        // values are not final
    }
    public void loop(){
//        if (voltageTimer.seconds() > 5) {
//            voltage = voltageSensor.getVoltage();
//            voltageTimer.reset();
//        }
//        state = profile.get(timer.time());
//        if (state.getV() != 0) {
//            targetLiftPosition = state.getX();
//        }
        liftPower = controller.calculate(liftPosition, targetLiftPosition);
        /* adjusts the needed lift power to adapt to the battery voltage. lower voltage calls for more
        lift power in order to run the slides at the same pace. Might need to adjust to 12 v during comp.
         */
    }

    public void read(){
        liftPosition = horizontalLift.getCurrentPosition() * SLIDE_TICK;
    }

    public void write(){
        horizontalLift.setPower(-liftPower);
    }

    public void setTargetLiftPosition(double v){targetLiftPosition = v;}

    public double getLiftPosition(){return liftPosition;}

    public void newProfile(double targetPos, double max_v, double max_a){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(getLiftPosition(), 0),
                new MotionState(targetPos, 0),
            max_v, max_a);
    }
}
