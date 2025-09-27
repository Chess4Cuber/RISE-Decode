package org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnSpindexerSystem {

    Gamepad gamepad1;
    RadahnSpindexer spindexer;
    RadahnTouchSensors touchSensors;
    public SpindexerStates spindexerState;
    boolean lastToggleA = false;

    public RadahnSpindexerSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        spindexer = new RadahnSpindexer(gamepad1, telemetry, hardwareMap);
        touchSensors = new RadahnTouchSensors(hardwareMap);

        this.gamepad1 = gamepad1;
        spindexerState = SpindexerStates.HOLE_0;

    }

    public void setPositions(){
        switch (spindexerState) {
            case HOLE_0:
                spindexer.setPosition(0);
                break;

            case HOLE_1:
                spindexer.setPosition(.7);
                break;

            case HOLE_2:
                spindexer.setPosition(-.7);
                break;
        }
    }

    public void controllerInput(){
        switch (spindexerState) {
            case HOLE_0:
                if (touchSensors.getTouchSensor(0)) {
                    spindexerState = SpindexerStates.HOLE_1;
                    touchSensors.setTouchSensor(0);
                }
                break;

            case HOLE_1:
                if (touchSensors.getTouchSensor(1)) {
                    spindexerState = SpindexerStates.HOLE_2;
                    touchSensors.setTouchSensor(1);
                }
                break;

            case HOLE_2:
                if (touchSensors.getTouchSensor(2)) {
                    spindexerState = SpindexerStates.HOLE_0;
                    touchSensors.setTouchSensor(2);
                }
                break;
        }

    }

    public void setHoleState(SpindexerStates state){
        spindexerState = state;
    }

}
