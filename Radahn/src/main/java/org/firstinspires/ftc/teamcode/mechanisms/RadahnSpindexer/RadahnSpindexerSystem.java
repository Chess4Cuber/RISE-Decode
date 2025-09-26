package org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexer;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnSpindexerSystem {

    Gamepad gamepad1;
    RadahnSpindexer spindexer;
    public SpindexerStates spindexerState;
    boolean lastToggleA = false;
    public RadahnSpindexerSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        spindexer = new RadahnSpindexer(gamepad1, telemetry, hardwareMap);

        this.gamepad1 = gamepad1;
        spindexerState = SpindexerStates.HOLE_1;

    }

    public void setPositions(){
        switch (spindexerState) {
            case HOLE_1:
                spindexer.setPosition(.47, 0);
                break;

            case HOLE_2:
                spindexer.setPosition(.7, 0);
                break;
        }
    }

    public void controllerInput(){
        switch (spindexerState) {
            case HOLE_1:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    spindexerState = SpindexerStates.HOLE_2;
                }

                break;
            case HOLE_2:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    spindexerState = SpindexerStates.HOLE_1;
                }
                break;
        }
        lastToggleA = gamepad1.a;
    }

}
