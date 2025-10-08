package org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.sensors.ColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnSpindexerSystem {

    Gamepad gamepad1;
    RadahnSpindexer spindexer;
    //RadahnTouchSensors touchSensors;
    RadahnColorSensors colorSensors;

    public SpindexerStates spindexerState;
//    public HoleColorStates colorState;

    public HoleColorStates HoleColor0, HoleColor1, HoleColor2;


    public RadahnSpindexerSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        spindexer = new RadahnSpindexer(gamepad1, telemetry, hardwareMap);
        //touchSensors = new RadahnTouchSensors(hardwareMap);
        colorSensors = new RadahnColorSensors(hardwareMap);

        this.gamepad1 = gamepad1;
        spindexerState = SpindexerStates.HOLE_0;
//        colorState = HoleColorStates.GREEN;

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

            case HOLE_OUTTAKE0:
                //spindexer.setPosition(.67);
                break;
            case HOLE_OUTTAKE1:
                //spindexer.setPosition(whatever);
                break;
            case HOLE_OUTTAKE2:
                //spindexer.setPosition(oieuboieurb);
                break;

        }
    }

    //TODO: Fix "a" value

    public void controllerInput(){
        switch (spindexerState) {
            case HOLE_0:

                //sees green
                if (colorSensors.seesColor(0, 0, 1, 0, .2f)){
                    HoleColor0 = HoleColorStates.GREEN;
                    spindexerState = SpindexerStates.HOLE_1;
                    display(HoleColor0);
                    //touchSensors.setTouchSensor(0);
                }

                //sees purple
                if (colorSensors.seesColor(0, .6f, 0, .9f, .2f)){
                    HoleColor0 = HoleColorStates.PURPLE;
                    spindexerState = SpindexerStates.HOLE_1;
                    display(HoleColor0);
                    //touchSensors.setTouchSensor(0);
                }
                break;

            case HOLE_1:
                if (colorSensors.seesColor(1, 0, 1, 0, .2f)){
                    HoleColor1 = HoleColorStates.GREEN;
                    spindexerState = SpindexerStates.HOLE_2;
                    display(HoleColor1);
                }

                if (colorSensors.seesColor(1, .6f, 0, .9f, .2f)){
                    HoleColor1 = HoleColorStates.PURPLE;
                    spindexerState = SpindexerStates.HOLE_2;
                    display(HoleColor1);
                }
                break;

            case HOLE_2:
                if (colorSensors.seesColor(2, 0, 1, 0, .2f)){
                    HoleColor2 = HoleColorStates.GREEN;
                    spindexerState = SpindexerStates.HOLE_0;
                    display(HoleColor2);
                }

                if (colorSensors.seesColor(2, .6f, 0, .9f, .2f)){
                    HoleColor2 = HoleColorStates.PURPLE;
                    spindexerState = SpindexerStates.HOLE_0;
                    display(HoleColor2);
                }
                break;

            case HOLE_OUTTAKE0:
                if(gamepad1.a){

                }

        }

    }

    public void setHoleState(SpindexerStates state){
        spindexerState = state;
    }
    public HoleColorStates getColorState(HoleColorStates color){
        return color;
    }



    public void display(HoleColorStates color){
        telemetry.addData("Spindexer State", spindexerState);
        telemetry.addData("Hole Color", getColorState(color));
        telemetry.update();
    }

}
