package org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnSpindexerSystem {

    Gamepad gamepad1;
    RadahnSpindexer spindexer;
    //RadahnTouchSensors touchSensors;
    RadahnColorSensors colorSensors;

    public SpindexerStates spindexerState;

    public RadahnSpindexerSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        spindexer = new RadahnSpindexer(gamepad1, telemetry, hardwareMap);
        //touchSensors = new RadahnTouchSensors(hardwareMap);
        colorSensors = new RadahnColorSensors(hardwareMap);

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

            case HOLE_OUTTAKE:
                //spindexer.setPosition(.67);
                break;
        }
    }

    //TODO: Fix "a" val

    public void controllerInput(){
        switch (spindexerState) {
            case HOLE_0:
                if (colorSensors.seesColor(0, 0, 256, 0, 0)){
                    spindexerState = SpindexerStates.HOLE_1;
                    //touchSensors.setTouchSensor(0);
                }

                if (colorSensors.seesColor(0, 128, 0, 128, 0)){
                    spindexerState = SpindexerStates.HOLE_1;
                    //touchSensors.setTouchSensor(0);
                }
                break;

            case HOLE_1:
                if (colorSensors.seesColor(0, 0, 256, 0, 0)){
                    spindexerState = SpindexerStates.HOLE_2;
                }

                if (colorSensors.seesColor(0, 128, 0, 128, 0)){
                    spindexerState = SpindexerStates.HOLE_2;
                }
                break;

            case HOLE_2:
                if (colorSensors.seesColor(0, 0, 256, 0, 0)){
                    spindexerState = SpindexerStates.HOLE_0;
                }

                if (colorSensors.seesColor(0, 128, 0, 128, 0)){
                    spindexerState = SpindexerStates.HOLE_0;
                }
                break;

//            case HOLE_OUTTAKE:
//                if () {
//                    spindexerState = SpindexerStates.HOLE_0;
//                }
//                break;
        }

    }

    public void setHoleState(SpindexerStates state){
        spindexerState = state;
    }

}
