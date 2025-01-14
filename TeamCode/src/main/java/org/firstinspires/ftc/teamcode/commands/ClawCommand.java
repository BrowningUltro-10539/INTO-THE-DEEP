package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ClawCommand extends CommandBase {
    private final ServoImplEx servo;

    private int state = 0;
    public ClawCommand(ServoImplEx clawServo, int state) {
        this.state = state;
        this.servo = clawServo;
    }
    public void flip(){
        this.servo.setPosition(state);
        state ^= 1;
    }
}