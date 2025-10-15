package org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnMotorIntakeSystem {
    Gamepad gamepad1;
    Telemetry telemetry;
    boolean lastToggleUp = false;
    boolean lastToggleA = false;
    RadahnMotorIntake outtake;
    public MotorIntakeStates outtakeState;

    public RadahnMotorIntakeSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {

        outtake = new RadahnMotorIntake(gamepad1, telemetry, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        outtakeState = MotorIntakeStates.RESTING;
    }

    public void setPositions(){
        switch(outtakeState){
            case RESTING:
                outtake.setPower(0);
                break;

            case INTAKING:
                outtake.setPower(1);
                break;

            case OUTTAKING:
                outtake.setPower(-1);
                break;
        }
    }

    public void controllerInput(){
        switch(outtakeState) {
            case RESTING:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    outtakeState = MotorIntakeStates.INTAKING;
                }
                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                    outtakeState = MotorIntakeStates.OUTTAKING;
                }
                break;

            case INTAKING:
                if((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    outtakeState = MotorIntakeStates.OUTTAKING;
                }
                if((gamepad1.a != lastToggleA) && gamepad1.a){
                    outtakeState = MotorIntakeStates.RESTING;
                }
                break;

            case OUTTAKING:
                if((gamepad1.a != lastToggleA) && gamepad1.a) {
                    outtakeState = MotorIntakeStates.INTAKING;
                }
                if((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    outtakeState = MotorIntakeStates.RESTING;
                }
                break;
        }
        lastToggleUp = gamepad1.dpad_up;
        lastToggleA = gamepad1.a;
    }

}
