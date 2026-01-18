package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurretSystem {

    Gamepad gamepad;
    Telemetry telemetry;
    RadahnTurret turret;

    public TurretStates turretState;

    // bumper edge detection memory
    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    double targetAngle = 0;

    // Manual step size
    public static final double STEP_ANGLE = 12;

    // Auto tracking correction gain
    public static final double TRACKING_K = 0.3;
    // ↓ lowered from 0.6 to prevent violent spinning on high gear ratio

    public RadahnTurretSystem(Gamepad gamepad, Telemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        turret = new RadahnTurret(gamepad, hardwareMap);

        // Start in auto tracking mode
        turretState = TurretStates.TRACKING;
        targetAngle = turret.getTurretAngle();
    }

    // Called from TeleOp each loop
    public void updateTargetAngle(double tx, boolean tagVisible) {

        switch (turretState) {

            case TRACKING:
                if (tagVisible) {
                    // Apply incremental correction
                    targetAngle += tx * TRACKING_K;
                }
                break;

            case MANUAL:
                // manual mode owns targetAngle
                break;
        }
    }

    public void controllerInput() {

        boolean leftBumper = gamepad.left_bumper;
        boolean rightBumper = gamepad.right_bumper;

        switch (turretState) {

            case TRACKING:

                if ((leftBumper != lastLeftBumper) && leftBumper) {
                    turretState = TurretStates.MANUAL;
                    targetAngle = turret.getTurretAngle() - STEP_ANGLE;
                }

                if ((rightBumper != lastRightBumper) && rightBumper) {
                    turretState = TurretStates.MANUAL;
                    targetAngle = turret.getTurretAngle() + STEP_ANGLE;
                }

                break;

            case MANUAL:

                // both bumpers → return to auto tracking
                if (leftBumper && rightBumper) {
                    turretState = TurretStates.TRACKING;
                }

                if ((leftBumper != lastLeftBumper) && leftBumper) {
                    targetAngle -= STEP_ANGLE;
                }

                if ((rightBumper != lastRightBumper) && rightBumper) {
                    targetAngle += STEP_ANGLE;
                }

                break;
        }

        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
    }

    public void setPositions() {

        switch (turretState) {

            case TRACKING:
                turret.setExtension(targetAngle);
                break;

            case MANUAL:
                turret.setExtension(targetAngle);
                break;
        }
    }

    public void setTelemetry() {
        telemetry.addData("Turret State", turretState);
        telemetry.addData("Turret Angle", turret.getTurretAngle());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Tag Tracking Gain", TRACKING_K);
        telemetry.addData("PID P", turret.slidesPID.P);
        telemetry.addData("PID I", turret.slidesPID.I);
        telemetry.addData("PID D", turret.slidesPID.D);
    }
}
