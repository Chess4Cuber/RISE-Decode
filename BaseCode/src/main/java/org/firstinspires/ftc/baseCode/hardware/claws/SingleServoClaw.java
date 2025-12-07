package org.firstinspires.ftc.baseCode.hardware.claws;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class SingleServoClaw {
    public Servo clawServo;

    public double openPosition;
    public double middlePosition;
    public double closedPosition;

    public enum ClawState{
        OPEN,
        MIDDLE,
        RESET,
        CLOSED
    }

    public ClawState clawState;

    public SingleServoClaw(String name, double openPosition, double middlePosition, double closedPosition, HardwareMap hardwareMap){
        clawServo = hardwareMap.get(Servo.class, name);

        this.openPosition = openPosition;
        this.middlePosition = middlePosition;
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

            case MIDDLE:
                setPosition(middlePosition);
                break;

            case RESET:
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

    public void middleClaw(){
        clawState = ClawState.MIDDLE;
        setPosition();
    }

    public void resetClaw(){
        clawState = ClawState.RESET;
        setPosition();
    }
    public void closeClaw(){
        clawState = ClawState.CLOSED;
        setPosition();
    }
}
