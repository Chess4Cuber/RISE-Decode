package org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnSpindexerSystem {

    Gamepad gamepad1;
    RadahnSpindexer spindexer;
    RadahnTouchSensors touchSensors;
    public SpindexerStates spindexerState;

    public RadahnSpindexerSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        //spindexer = new RadahnSpindexer(gamepad1, telemetry, hardwareMap);
        touchSensors = new RadahnTouchSensors(hardwareMap);

        this.gamepad1 = gamepad1;
        spindexerState = SpindexerStates.HOLE_0;

    }

    public void setPositions(){
        switch (spindexerState) {
            case HOLE_0:
                //spindexer.setPosition(0);
                break;

            case HOLE_1:
                //spindexer.setPosition(.7);
                break;

            case HOLE_2:
                //spindexer.setPosition(-.7);
                break;
        }
    }

    public void controllerInput(){
        switch (spindexerState) {
            case HOLE_0:
                if (touchSensors.getTouchSensor(0)) {
                    spindexerState = SpindexerStates.HOLE_1;
                    telemetry.addData("hole 0", touchSensors.getTouchSensor(0));
                    //touchSensors.setTouchSensor(0);
                }
                break;

            case HOLE_1:
                if (touchSensors.getTouchSensor(1)) {
                    spindexerState = SpindexerStates.HOLE_2;
                    telemetry.addData("hole 1", touchSensors.getTouchSensor(1));
                    //touchSensors.setTouchSensor(1);
                }
                break;

            case HOLE_2:
                if (touchSensors.getTouchSensor(2)) {
                    spindexerState = SpindexerStates.HOLE_0;
                    telemetry.addData("hole 2", touchSensors.getTouchSensor(2));
                    //touchSensors.setTouchSensor(2);
                }
                break;
        }

    }

    public void setHoleState(SpindexerStates state){
        spindexerState = state;
    }

}
