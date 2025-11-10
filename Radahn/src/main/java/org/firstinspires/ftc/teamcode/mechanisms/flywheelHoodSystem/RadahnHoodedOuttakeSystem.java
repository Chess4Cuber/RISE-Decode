package org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnHoodedOuttakeSystem {

    private final RadahnHoodedOuttake hoodedOuttake;
    private final Gamepad gamepad1;
    private final Telemetry telemetry;

    private boolean lastToggleB = false;
    private boolean lastToggleBack = false;

    private TurretHoodStates outtakeState;

    // Latest distance to AprilTag (in inches)
    private double tagDistanceInches = 0;

    public RadahnHoodedOuttakeSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        hoodedOuttake = new RadahnHoodedOuttake(gamepad1, telemetry, hardwareMap);
        outtakeState = TurretHoodStates.RESTING;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    /** Updates motor and hood positions based on current tag distance */
    public void setPositions() {
        double targetRPM = computeMotorVelocity(tagDistanceInches) * 60.0 / hoodedOuttake.motors[0].TICKS_PER_REV;
        double targetHoodPosition = computeHoodPosition(tagDistanceInches);

        switch (outtakeState) {
            case RESTING:
                hoodedOuttake.setPower(0);
                break;

            case INTAKING:
                hoodedOuttake.setVelocityRPM(targetRPM);
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;

            case OUTTAKING:
                hoodedOuttake.setVelocityRPM(-targetRPM);
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;
        }

        displayTelemetry(targetRPM, targetHoodPosition);
    }

    /** Handles button-based state changes */
    public void controllerInput() {
        switch (outtakeState) {
            case RESTING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    outtakeState = TurretHoodStates.INTAKING;
                }

                if ((gamepad1.back != lastToggleBack) && gamepad1.back) {
                    outtakeState = TurretHoodStates.OUTTAKING;
                }
                break;

            case INTAKING:
                if ((gamepad1.back != lastToggleBack) && gamepad1.back){
                    outtakeState = TurretHoodStates.OUTTAKING;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    outtakeState = TurretHoodStates.RESTING;
                }
                break;

            case OUTTAKING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    outtakeState = TurretHoodStates.INTAKING;
                }

                if ((gamepad1.back != lastToggleBack) && gamepad1.back){
                    outtakeState = TurretHoodStates.RESTING;
                }
                break;
        }

        lastToggleBack = gamepad1.back;
        lastToggleB = gamepad1.b;
    }

    public void setMotorOuttakeState(TurretHoodStates state) {
        this.outtakeState = state;
    }

    /** Computes motor velocity based on AprilTag distance (ticks/sec) */
    public double computeMotorVelocity(double distanceInches) {
        if (distanceInches <= 0) return 0;

        double minDist = 6;
        double maxDist = 36;
        double minRPM = 500;
        double maxRPM = 3000;

        double slope = (maxRPM - minRPM) / (maxDist - minDist);
        double targetRPM = minRPM + slope * (distanceInches - minDist);
        targetRPM = Math.max(minRPM, Math.min(maxRPM, targetRPM));

        // Convert RPM → ticks/sec
        return targetRPM * hoodedOuttake.motors[0].TICKS_PER_REV / 60.0;
    }

    /** Computes hood servo position based on AprilTag distance */
    public double computeHoodPosition(double distanceInches) {
        if (distanceInches <= 0) return 0.5;

        double minDist = 6;
        double maxDist = 36;
        double minPos = 0.8;
        double maxPos = 0;

        double normalized = (distanceInches - minDist) / (maxDist - minDist);
        normalized = Math.max(0, Math.min(1, normalized));

        // Quadratic curve for smooth hood adjustment
        double position = minPos + (maxPos - minPos) * (normalized * normalized);
        return Math.max(minPos, Math.min(maxPos, position));
    }

    /** Updates the latest measured distance from the AprilTag */
    public void updateDistance(double distanceInches) {
        this.tagDistanceInches = distanceInches;
    }

    /** Telemetry for tuning */
    private void displayTelemetry(double targetRPM, double targetHoodPosition) {
        telemetry.addData("Outtake State", outtakeState);
        telemetry.addData("Distance (in)", "%.2f", tagDistanceInches);
        telemetry.addData("Motor Target RPM", "%.1f", targetRPM);
        telemetry.addData("Hood Target Pos", "%.3f", targetHoodPosition);
    }
}
