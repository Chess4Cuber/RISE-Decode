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
    double lastValidDistance = 0;   // <-- NEW

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

    // NEW: tagVisible decides whether to update stored distance
    public void updateDistance(double distanceInches, boolean tagVisible) {
        if (tagVisible) {
            lastValidDistance = distanceInches;
        }
        tagDistanceInches = lastValidDistance;
    }

    public void setPositions() {

        double targetFlywheelCommand = computeFlywheelPower(tagDistanceInches);
        double targetHoodPosition = computeHoodPosition(tagDistanceInches);

        switch (outtakeState) {
            case RESTING:
                hoodedOuttake.setFlywheelPower(0);
                break;

            case OUTTAKING:
                hoodedOuttake.setFlywheelPower(targetFlywheelCommand); // PID is applied inside RadahnHoodedOuttake#setFlywheelPower
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;

            case INTAKING:
                hoodedOuttake.setFlywheelPower(-targetFlywheelCommand); // PID is applied inside RadahnHoodedOuttake#setFlywheelPower
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;
        }
    }

    public void controllerInput() {
        switch (outtakeState) {
            case RESTING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b)
                    outtakeState = TurretHoodStates.OUTTAKING;

                if ((gamepad1.back != lastToggleBack) && gamepad1.back)
                    outtakeState = TurretHoodStates.INTAKING;
                break;

            case OUTTAKING:
                if ((gamepad1.back != lastToggleBack) && gamepad1.back)
                    outtakeState = TurretHoodStates.INTAKING;

                if ((gamepad1.b != lastToggleB) && gamepad1.b)
                    outtakeState = TurretHoodStates.RESTING;
                break;

            case INTAKING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b)
                    outtakeState = TurretHoodStates.OUTTAKING;

                if ((gamepad1.back != lastToggleBack) && gamepad1.back)
                    outtakeState = TurretHoodStates.RESTING;
                break;
        }

        lastToggleBack = gamepad1.back;
        lastToggleB = gamepad1.b;
    }

    // ---- NEW POWER CURVE ----
    // Outputs motor power (0 to 1)
    public double computeFlywheelPower(double distanceInches) {

        if(distanceInches>=15.5){
            return .6;
        }

        // Tuned to ~half previous close-shot speed
        double a = 0.001;
        double b = 0.015;
        double c = 0.15;

        double power = (a * distanceInches * distanceInches) +
                (b * distanceInches) + c;

        // Clamp
        power = Math.max(0.0, Math.min(1.0, power));
        return power;
    }

    public double computeHoodPosition(double distanceInches) {
        if (distanceInches < 10){
            return 0.0;
        } else if (distanceInches < 15.5) {
            return .28;
        }

        return .2;
    }

    public void setMotorOuttakeState(TurretHoodStates state) {
        this.outtakeState = state;
    }

    private void displayTelemetry() {
        telemetry.addData("Outtake State", outtakeState);
        telemetry.addData("Shooter Distance Used", "%.1f", tagDistanceInches);
        telemetry.addData("Flywheel Power", "%.2f", computeFlywheelPower(tagDistanceInches));
        telemetry.addData("Hood Pos", "%.2f", hoodedOuttake.getHoodPosition());
    }
}
