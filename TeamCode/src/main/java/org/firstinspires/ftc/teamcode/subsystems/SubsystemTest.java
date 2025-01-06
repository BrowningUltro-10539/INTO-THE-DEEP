package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class SubsystemTest{
    private ServoImplEx leftServo, rightServo;
    public SubsystemTest(HardwareMap hardwareMap, boolean isAuto){
        leftServo = hardwareMap.get(ServoImplEx.class, "leftServo");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightServo");
    }

    public void setPos(double pos){
           leftServo.setPosition(pos);
           rightServo.setPosition(1 - pos);
    }
    public void read(){}
    public void write(){}
    public void loop(){}

}
