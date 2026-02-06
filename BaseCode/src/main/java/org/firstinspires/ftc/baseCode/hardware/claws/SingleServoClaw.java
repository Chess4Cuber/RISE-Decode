package org.firstinspires.ftc.baseCode.hardware.claws;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class SingleServoClaw {
    public Servo clawServo;

    public double openPosition;
    public double closedPosition;

    public enum ClawState{
        OPEN,
        CLOSED
    }

    public ClawState clawState;

    public SingleServoClaw(String name, double openPosition, double closedPosition, HardwareMap hardwareMap){
        clawServo = hardwareMap.get(Servo.class, name);

        this.openPosition = openPosition;
        this.closedPosition = closedPosition;

        clawState = ClawState.OPEN;
    }

    public void setPosition(double position){
        clawServo.setPosition(position);
    }

    public void setPosition(){
        switch (clawState){
            case OPEN:
                setPosition(openPosition);
                break;

            case CLOSED:
                setPosition(closedPosition);
                break;
        }
    }

    public void openClaw(){
        clawState = ClawState.OPEN;
        setPosition();
    }

    public void closeClaw(){
        clawState = ClawState.CLOSED;
        setPosition();
    }
}