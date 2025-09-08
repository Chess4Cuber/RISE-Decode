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
    boolean lastToggleX = false;
    boolean lastToggleB = false;

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
                intake.toggleIntake();
                break;
        }
    }

    public void controllerInput() {
        switch (intakeState) {
            case RESTING:
                if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                    intakeState = IntakeStates.INTAKING;
                }
                if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                    intakeState = IntakeStates.OUTTAKING;
                }
                break;
        }
        lastToggleX = gamepad1.x;
        lastToggleB = gamepad1.b;
    }
}
