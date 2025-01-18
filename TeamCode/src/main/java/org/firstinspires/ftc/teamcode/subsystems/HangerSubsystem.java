package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangerSubsystem{
    public final MotorEx leftArm;
    public final MotorEx rightArm;
    private final PIDController controller;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double kG = -5;
    public double profile;

    public PIDController liftController;

    public double currentLiftPosition;
    public double targetLiftPosition;
    public double liftPower;
    //change ticks of slide motors(check specks)
    private final double slidesTickPerInch = 2 * Math.PI * 0.701771654 / 145.1;

    private boolean isAuto = false;


    public HangerSubsystem(HardwareMap hardwareMap, boolean isAuto){
        this.leftArm = new MotorEx(hardwareMap, "liftOne");
        this.rightArm = new MotorEx(hardwareMap, "liftTwo");
        this.controller = new PIDController(P, I, D);

        this.isAuto = isAuto;
    }

    public void read(){
        currentLiftPosition = rightArm.encoder.getPosition() * slidesTickPerInch ;
    }

    public void loop() {
        liftPower = controller.calculate(currentLiftPosition, targetLiftPosition) + kG;
    }
    public double getLiftPos(){
        return currentLiftPosition;
    }
    public void write(){
        leftArm.set(liftPower);
        rightArm.set(-liftPower);
    }
    public void setTargetLiftPosition(double value){
        targetLiftPosition = value;
    }

    public void newLiftProfile(double targetLiftPosition){
        profile = liftController.calculate(getLiftPos(), targetLiftPosition);
    }
}