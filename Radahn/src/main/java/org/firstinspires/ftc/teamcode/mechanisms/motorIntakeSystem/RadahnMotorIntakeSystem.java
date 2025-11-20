package org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnMotorIntakeSystem {
    Gamepad gamepad1;
    Telemetry telemetry;
    boolean lastToggleUp = false;
    boolean lastToggleA = false;
    RadahnMotorIntake intake;
    public MotorIntakeStates intakeState;

    public RadahnMotorIntakeSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {

        intake = new RadahnMotorIntake(gamepad1, telemetry, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        intakeState = MotorIntakeStates.RESTING;

    }

    public void setPositions(){
        switch(intakeState){
            case RESTING:
                intake.setPower(0);
                break;

            case INTAKING:
                intake.setPower(-1);
                break;

            case OUTTAKING:
                intake.setPower(1);
                break;
        }
    }

    public void controllerInput(){
        switch(intakeState) {
            case RESTING:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    intakeState = MotorIntakeStates.INTAKING;
                }
                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                    intakeState = MotorIntakeStates.OUTTAKING;
                }
                break;

            case INTAKING:
                if((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    intakeState = MotorIntakeStates.OUTTAKING;
                }
                if((gamepad1.a != lastToggleA) && gamepad1.a){
                    intakeState = MotorIntakeStates.RESTING;
                }
                break;

            case OUTTAKING:
                if((gamepad1.a != lastToggleA) && gamepad1.a) {
                    intakeState = MotorIntakeStates.INTAKING;
                }
                if((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    intakeState = MotorIntakeStates.RESTING;
                }
                break;
        }
        lastToggleUp = gamepad1.dpad_up;
        lastToggleA = gamepad1.a;
    }

}