package org.firstinspires.ftc.teamcode.mechanisms.motorIntake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.intakeSystem.IntakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.OptimusArm;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.OptimusSlides;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.SlidesStates;

public class MotorOptimusIntakeSystem {
    Gamepad gamepad1;
    Telemetry telemetry;
    boolean lastToggleUp = false;
    boolean lastToggleDown = false;
    boolean lastToggleRight = false;
    MotorOptimusIntake intake;
    public IntakeStates intakeState;

    public MotorOptimusIntakeSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {

        intake = new MotorOptimusIntake(gamepad1, telemetry, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        intakeState = IntakeStates.RESTING;
    }

    public void setPositions() {
        switch (intakeState) {
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

    public void controllerInput() {
        switch (intakeState) {
            case RESTING:
                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                    intakeState = IntakeStates.INTAKING;
                }
                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                    intakeState = IntakeStates.OUTTAKING;
                }
                break;

            case INTAKING:
                if((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    intakeState = IntakeStates.OUTTAKING;;
                }
                if((gamepad1.dpad_right != lastToggleRight) && gamepad1.dpad_right){
                    intakeState = IntakeStates.RESTING;
                }
                break;

            case OUTTAKING:
                if((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                    intakeState = IntakeStates.INTAKING;
                }
                if((gamepad1.dpad_right != lastToggleRight) && gamepad1.dpad_right){
                    intakeState = IntakeStates.RESTING;
                }
                break;
        }
        lastToggleUp = gamepad1.dpad_up;
        lastToggleDown = gamepad1.dpad_down;
        lastToggleRight = gamepad1.dpad_right;
    }
}
