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

    public void setPositions() {
        // FIXED — compute Motor RPM correctly, not double conversion
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

        displayTelemetry(targetRPM, targetHoodPosition);
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

    public void setMotorOuttakeState(TurretHoodStates state) {
        this.outtakeState = state;
    }

    public double computeMotorVelocity(double distanceInches) {

        double minDist = 30;
        double maxDist = 120;

        // tune these
        double minRPM = 1000;
        double maxRPM = 4700;

        double clampedDist = Math.max(minDist, Math.min(maxDist, distanceInches));
        double normalized = (clampedDist - minDist) / (maxDist - minDist);  // 0..1

        return minRPM + (maxRPM - minRPM) * Math.pow(normalized,.25);

    }

    // FIXED — positive range, no double-negation needed
    public double computeHoodPosition(double distanceInches) {

//        double minDist = 6;
//        double maxDist = 120;
//
//        double minPos = 0.0;
//        double maxPos = 1;
//
//        double clampedDist = Math.max(minDist, Math.min(maxDist, distanceInches));
//        double normalized = (clampedDist - minDist) / (maxDist - minDist);
//
//        double targetHoodPosition = minPos + (maxPos - minPos) * (normalized * normalized);
//        return Math.max(minPos, Math.min(maxPos, targetHoodPosition));
        return .3;
    }

//    public double computeHoodPositionSimple(double distanceInches){
//        if(distance <)
//    }

    public void updateDistance(double distanceInches) {
        this.tagDistanceInches = distanceInches;
    }

    private void displayTelemetry(double targetRPM, double targetHoodPosition) {
        telemetry.addData("Outtake State", outtakeState);
        telemetry.addData("Distance (in)", "%.2f", tagDistanceInches);
        telemetry.addData("Motor Target RPM", "%.1f", targetRPM);
        telemetry.addData("Motor Actual RPM", "%.1f", hoodedOuttake.getRPMMotor());
        telemetry.addData("Hood Target Pos", "%.3f", targetHoodPosition);
        telemetry.addData("Servo Actual Pos", hoodedOuttake.getHoodPosition());

    }
}
