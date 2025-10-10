package org.firstinspires.ftc.teamcode.mechanisms.motorOuttake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnMotorOuttakeSystem {
    Gamepad gamepad1;
    Telemetry telemetry;
    boolean lastToggleBack = false;
    boolean lastToggleB = false;
    RadahnMotorOuttake outtake;
    public MotorOuttakeStates outtakeState;

    public RadahnMotorOuttakeSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {

        outtake = new RadahnMotorOuttake(gamepad1, telemetry, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        outtakeState = MotorOuttakeStates.RESTING;
    }

    public void setPositions() {
        switch (outtakeState) {
            case RESTING:
                outtake.setPower(0);
                break;

            case INTAKING:
                outtake.setPower(.6);
                break;

            case OUTTAKING:
                outtake.setPower(-.6);
                break;
        }

    }

    public void controllerInput() {
        switch (outtakeState) {
            case RESTING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                    outtakeState = MotorOuttakeStates.INTAKING;
                }
                if ((gamepad1.back != lastToggleBack) && gamepad1.back) {
                    outtakeState = MotorOuttakeStates.OUTTAKING;
                }
                break;

            case INTAKING:
                if((gamepad1.back != lastToggleBack) && gamepad1.back){
                    outtakeState = MotorOuttakeStates.OUTTAKING;;
                }
                if((gamepad1.b != lastToggleB) && gamepad1.b){
                    outtakeState = MotorOuttakeStates.RESTING;
                }
                break;

            case OUTTAKING:
                if((gamepad1.b != lastToggleB) && gamepad1.b) {
                    outtakeState = MotorOuttakeStates.INTAKING;
                }
                break;

        }

        lastToggleBack = gamepad1.back;
        lastToggleB = gamepad1.b;
    }

    public void setMotorOuttakeState(MotorOuttakeStates state){
        outtakeState = state;
    }

}
