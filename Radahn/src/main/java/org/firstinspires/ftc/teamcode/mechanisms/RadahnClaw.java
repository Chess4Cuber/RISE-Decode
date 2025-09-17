package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.claws.SingleServoClaw;

public class RadahnClaw extends SingleServoClaw {
    Gamepad gamepad1;
    boolean lastToggleY = false;
    public RadahnClaw(Gamepad gamepad1, HardwareMap hardwareMap){
        super("clawServo", .75, .4, hardwareMap);

        this.gamepad1 = gamepad1;
    }

    public void toggleClaw(){
        switch (clawState){
            case OPEN:
                if ((gamepad1.y != lastToggleY) && gamepad1.y){
                    closeClaw();
                }
                break;
            case CLOSED:
                if ((gamepad1.y != lastToggleY) && gamepad1.y){
                    openClaw();
                }
                break;
        }
        lastToggleY = gamepad1.y;
    }
}
