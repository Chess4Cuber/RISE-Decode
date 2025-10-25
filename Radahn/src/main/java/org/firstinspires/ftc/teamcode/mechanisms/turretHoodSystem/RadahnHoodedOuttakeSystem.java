package org.firstinspires.ftc.teamcode.mechanisms.turretHoodSystem;

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

    public void updateDistance(double distanceInches) {
        this.tagDistanceInches = distanceInches;
    }

    public void setPositions() {
        double targetVelocity = computeMotorVelocity(tagDistanceInches);
        double targetHoodPosition = computeHoodPosition(tagDistanceInches);

        switch (outtakeState) {
            case RESTING:
                hoodedOuttake.setPower(0);
                break;

            case INTAKING:
                //hoodedOuttake.setVelocity(targetVelocity);
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;

            case OUTTAKING:
                //hoodedOuttake.setVelocity(-targetVelocity);
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;
        }

        telemetry.addData("Outtake State", outtakeState);
        telemetry.addData("Distance (in)", "%.2f", tagDistanceInches);
        telemetry.addData("Motor Target Vel", "%.3f", targetVelocity);
        telemetry.addData("Hood Target Pos", "%.3f", targetHoodPosition);
    }

    public void controllerInput() {
        switch (outtakeState) {
            case RESTING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b)
                    outtakeState = TurretHoodStates.INTAKING;
                if ((gamepad1.back != lastToggleBack) && gamepad1.back)
                    outtakeState = TurretHoodStates.OUTTAKING;
                break;

            case INTAKING:
                if ((gamepad1.back != lastToggleBack) && gamepad1.back)
                    outtakeState = TurretHoodStates.OUTTAKING;
                if ((gamepad1.b != lastToggleB) && gamepad1.b)
                    outtakeState = TurretHoodStates.RESTING;
                break;

            case OUTTAKING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b)
                    outtakeState = TurretHoodStates.INTAKING;
                break;
        }

        lastToggleBack = gamepad1.back;
        lastToggleB = gamepad1.b;
    }

    public void setMotorOuttakeState(TurretHoodStates state) {
        this.outtakeState = state;
    }

    /**
     * Maps AprilTag distance (in inches) → desired motor velocity.
     * TODO: tune function experimentally.
     */
    private double computeMotorVelocity(double distanceInches) {
        if (distanceInches <= 0) return 0;

        // Example placeholder: linear relationship
        // velocity increases from 400 -> 1200 ticks/sec between 6" and 36"
        double minDist = 6, maxDist = 36;
        double minVel = 400, maxVel = 1200;
        double slope = (maxVel - minVel) / (maxDist - minDist);

        double velocity = minVel + slope * (distanceInches - minDist);
        return Math.max(minVel, Math.min(maxVel, velocity));
    }

    /**
     * Maps AprilTag distance (in inches) → hood servo position.
     * TODO: tune function experimentally.
     */
    private double computeHoodPosition(double distanceInches) {
        if (distanceInches <= 0) return 0.5; // default neutral

        // Example placeholder: servo goes from 0.2 → 0.8 as distance increases
        double minDist = 6, maxDist = 36;
        double minPos = 0.2, maxPos = 0.8;
        double slope = (maxPos - minPos) / (maxDist - minDist);

        double position = minPos + slope * (distanceInches - minDist);
        return Math.max(minPos, Math.min(maxPos, position));
    }
}
