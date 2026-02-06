package org.firstinspires.ftc.teamcode.mechanisms.RadahnTransfer;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RadahnServoIntakeSystem {

    Gamepad gamepad1;
    RadahnServoIntake intake;
    boolean lastToggleUp = false;
    boolean lastToggleY = false;

    public IntakeStates intakeState;

    public RadahnServoIntakeSystem(Gamepad gamepad1, HardwareMap hardwareMap){
        intake = new RadahnServoIntake(hardwareMap);
        intakeState = IntakeStates.RESTING;
        this.gamepad1 = gamepad1;
    }

    public void setPositions(){
        switch(intakeState){
            case RESTING:
                intake.setPower(0);
                break;

            case INTAKING:
                intake.setPower(1);
                break;

            case OUTTAKING:
                intake.setPower(-1);
                break;
        }
    }

    public void controllerInput(){
        switch(intakeState) {
            case RESTING:
                if ((gamepad1.y != lastToggleY) && gamepad1.y) {
                    intakeState = IntakeStates.INTAKING;
                }
                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                    intakeState = IntakeStates.OUTTAKING;
                }
                break;

            case INTAKING:
                if((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    intakeState = IntakeStates.OUTTAKING;
                }
                if((gamepad1.y != lastToggleY) && gamepad1.y){
                    intakeState = IntakeStates.RESTING;
                }
                break;

            case OUTTAKING:
                if((gamepad1.y != lastToggleY) && gamepad1.y) {
                    intakeState = IntakeStates.INTAKING;
                }
                if((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    intakeState = IntakeStates.RESTING;
                }
                break;
        }
         lastToggleUp = gamepad1.dpad_up;
         lastToggleY = gamepad1.dpad_down;
     }

     public void setServoIntakeState(IntakeStates state){
        intakeState = state;
     }


}
