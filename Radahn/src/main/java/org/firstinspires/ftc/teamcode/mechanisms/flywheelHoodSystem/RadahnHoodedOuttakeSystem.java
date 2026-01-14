package org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RadahnHoodedOuttakeSystem {

    RadahnHoodedOuttake hoodedOuttake;
    Gamepad gamepad1;
    Telemetry telemetry;

    boolean lastToggleB = false;
    boolean lastToggleBack = false;

    TurretHoodStates outtakeState;

    double tagDistanceInches = 0;

    public RadahnHoodedOuttakeSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        hoodedOuttake = new RadahnHoodedOuttake(gamepad1, telemetry, hardwareMap);
        outtakeState = TurretHoodStates.RESTING;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void update() {
        controllerInput();
        setPositions();
        displayTelemetry();
    }

    public void setPositions() {

        double targetRPM = computeMotorVelocity(tagDistanceInches);
        double targetHoodPosition = computeHoodPosition(tagDistanceInches);

        switch (outtakeState) {
            case RESTING:
                hoodedOuttake.setPower(0);
                break;

            case OUTTAKING:
                hoodedOuttake.setVelocityRPM(targetRPM);
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;

            case INTAKING:
                hoodedOuttake.setVelocityRPM(-targetRPM);
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;
        }
    }

    public void controllerInput() {
        switch (outtakeState) {
            case RESTING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                    outtakeState = TurretHoodStates.OUTTAKING;
                }
                if ((gamepad1.back != lastToggleBack) && gamepad1.back) {
                    outtakeState = TurretHoodStates.INTAKING;
                }
                break;

            case OUTTAKING:
                if ((gamepad1.back != lastToggleBack) && gamepad1.back) {
                    outtakeState = TurretHoodStates.INTAKING;
                }
                if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                    outtakeState = TurretHoodStates.RESTING;
                }
                break;

            case INTAKING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                    outtakeState = TurretHoodStates.OUTTAKING;
                }
                if ((gamepad1.back != lastToggleBack) && gamepad1.back) {
                    outtakeState = TurretHoodStates.RESTING;
                }
                break;
        }

        lastToggleBack = gamepad1.back;
        lastToggleB = gamepad1.b;
    }

    public void updateDistance(double distanceInches) {
        this.tagDistanceInches = distanceInches;
    }

    public double computeMotorVelocity(double distanceInches) {
        // Quadratic: rpm = a*x^2 + b*x + c
        double a = 50;    // example coefficient
        double b = 200;   // example coefficient
        double c = 1500;  // example coefficient

        double rpm = (a * distanceInches * distanceInches) + (b * distanceInches) + c;

        // Clamp to motor limits
        rpm = Math.max(0, Math.min(6000, rpm));
        return rpm;
    }

    public double computeHoodPosition(double distanceInches) {
        if (distanceInches < 30) {
            return 0.0;
        } else if (distanceInches < 60) {
            return 0.5;
        } else {
            return 1.0;
        }
    }

    public void setMotorOuttakeState(TurretHoodStates state) {
        this.outtakeState = state;
    }

    private void displayTelemetry() {
        double targetRPM = computeMotorVelocity(tagDistanceInches);
        double targetHoodPos = computeHoodPosition(tagDistanceInches);

        telemetry.addData("Outtake State", outtakeState);
        telemetry.addData("Distance (in)", "%.2f", tagDistanceInches);
        telemetry.addData("Motor Target RPM", "%.1f", targetRPM);
        telemetry.addData("Motor Actual RPM", "%.1f", hoodedOuttake.getRPMMotor());
        telemetry.addData("Hood Target Pos", "%.3f", targetHoodPos);
        telemetry.addData("Hood Actual Pos", "%.3f", hoodedOuttake.getHoodPosition());
    }

}
