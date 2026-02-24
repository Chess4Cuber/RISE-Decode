package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurretSystem {

    Gamepad gamepad1;
    Telemetry telemetry;
    RadahnTurret turret;

    public TurretStates turretState;

    private static final double LEFT_LIMIT = -95.0;   // Maximum left rotation
    private static final double RIGHT_LIMIT = 110.0;   // Maximum right rotation

    private static final double MANUAL_STEP = 10.0;

    // how aggressively to correct for tx
    private static final double AUTO_TRACKING_GAIN = 0.2;

    private static final double CAMERA_DEADBAND = 7;
    private static final double MAX_POWER = 0.5;

    // Unwinding feature - triggers when this close to limit
    private static final double UNWIND_THRESHOLD = 10.0;  // degrees from limit

    private double targetAngle = 0;

    private double tx = 0;
    private boolean targetVisible = false;

    // Unwinding state
    private boolean isUnwinding = false;
    private double unwindTargetAngle = 0;

    private boolean lastLB = false;
    private boolean lastRB = false;

    public RadahnTurretSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        turret = new RadahnTurret(gamepad1, telemetry, hardwareMap);
        turretState = TurretStates.AUTO_AIM;
        targetAngle = turret.getAngleDegrees();
    }

    public void updateLimelight(double tx, boolean targetVisible) {
        this.tx = tx;
        this.targetVisible = targetVisible;
    }

    public void update() {
        controllerInput();
        setPositions();
        displayTelemetry();
    }

    public void controllerInput() {

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            if (turretState != TurretStates.AUTO_AIM) {
                turretState = TurretStates.AUTO_AIM;
                isUnwinding = false;  // Cancel any unwinding when returning to AUTO
            }
        }

        else if ((gamepad1.left_bumper != lastLB) && gamepad1.left_bumper) {
            turretState = TurretStates.MANUAL;
            targetAngle -= MANUAL_STEP;

            targetAngle = clamp(targetAngle, LEFT_LIMIT, RIGHT_LIMIT);
            isUnwinding = false;
        }

        else if ((gamepad1.right_bumper != lastRB) && gamepad1.right_bumper) {
            turretState = TurretStates.MANUAL;
            targetAngle += MANUAL_STEP;
            // Clamp to limits
            targetAngle = clamp(targetAngle, LEFT_LIMIT, RIGHT_LIMIT);
            isUnwinding = false;
        }

        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
    }


    public void setPositions() {

        double currentAngle = turret.getAngleDegrees();
        double appliedPower = 0;

        switch (turretState) {

            case RESTING:
                turret.stop();
                isUnwinding = false;
                break;

            case MANUAL:
                appliedPower = turret.setTargetAngle(targetAngle, MAX_POWER);
                break;

            case AUTO_AIM:

                // Check if we're currently unwinding
                if (isUnwinding) {
                    // Drive to unwind target (opposite limit)
                    appliedPower = turret.setTargetAngle(unwindTargetAngle, MAX_POWER);

                    // Check if unwind is complete
                    if (turret.isAtTarget(unwindTargetAngle)) {
                        isUnwinding = false;
                        // Unwinding complete - resume normal tracking
                    }
                }
                else if (targetVisible) {
                    // Normal tracking mode with deadband
                    if (Math.abs(tx) > CAMERA_DEADBAND) {
                        targetAngle += tx * AUTO_TRACKING_GAIN;
                    }

                    // Check if we should trigger an unwind
                    if (shouldUnwind(currentAngle, targetAngle)) {
                        triggerUnwind(currentAngle, targetAngle);
                    }
                    else {
                        // Normal operation - clamp to limits
                        targetAngle = clamp(targetAngle, LEFT_LIMIT, RIGHT_LIMIT);
                        appliedPower = turret.setTargetAngle(targetAngle, MAX_POWER);
                    }
                }
                else {
                    // Target not visible - hold position
                    appliedPower = turret.setTargetAngle(targetAngle, MAX_POWER);
                }
                break;
        }

        if (!isUnwinding) {
            applySoftLimits(appliedPower, currentAngle);
        }
    }

    private void applySoftLimits(double power, double angle) {
        if (angle <= LEFT_LIMIT && power < 0) {
            turret.setPower(0);
            return;
        }

        if (angle >= RIGHT_LIMIT && power > 0) {
            turret.setPower(0);
            return;
        }

    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Determines if turret should unwind based on current and target positions
     *
     * Triggers unwind when:
     * 1. Target is on opposite side of center from current position
     * 2. Current position is near a limit
     * 3. Going around would allow continuous tracking
     */
    private boolean shouldUnwind(double current, double target) {

        // Near right limit and target is on left side?
        if (current > (RIGHT_LIMIT - UNWIND_THRESHOLD) && target < 0) {
            return true;
        }

        // Near left limit and target is on right side?
        if (current < (LEFT_LIMIT + UNWIND_THRESHOLD) && target > 0) {
            return true;
        }

        // Target would exceed limits?
        if (target > RIGHT_LIMIT || target < LEFT_LIMIT) {
            return true;
        }

        return false;
    }

    private void triggerUnwind(double current, double target) {
        isUnwinding = true;

        // Determine which direction to unwind
        if (current > 0) {
            // Currently on right side - unwind through left
            unwindTargetAngle = LEFT_LIMIT;
        }
        else {
            // Currently on left side - unwind through right
            unwindTargetAngle = RIGHT_LIMIT;
        }
    }

    public void setState(TurretStates state) {
        this.turretState = state;
    }

    public void setTargetAngle(double angle) {
        this.targetAngle = clamp(angle, LEFT_LIMIT, RIGHT_LIMIT);
    }

    public TurretStates getState() {
        return turretState;
    }

    public boolean isReadyToShoot() {
        // Not ready while unwinding
        if (isUnwinding) return false;

        return turret.isAtTarget(targetAngle) && Math.abs(tx) <= CAMERA_DEADBAND;
    }


    private void displayTelemetry() {
        double currentAngle = turret.getAngleDegrees();
        double error = turret.getError(isUnwinding ? unwindTargetAngle : targetAngle);
        boolean atTarget = turret.isAtTarget(isUnwinding ? unwindTargetAngle : targetAngle);
        boolean withinDeadband = Math.abs(tx) <= CAMERA_DEADBAND;

        telemetry.addData("─── TURRET STATUS ───", "");
        telemetry.addData("Mode", turretState);

        if (isUnwinding) {
            telemetry.addData("UNWINDING", "To %.1f°", unwindTargetAngle);
        }
        else {
            telemetry.addData("At Target", atTarget ? "YES" : "NO");
            telemetry.addData("Centered", withinDeadband ? "YES" : "NO");
        }

        telemetry.addData("─── ANGLES ───", "");
        telemetry.addData("Current Angle", "%.1f°", currentAngle);
        telemetry.addData("Target Angle", "%.1f°", isUnwinding ? unwindTargetAngle : targetAngle);
        telemetry.addData("Error", "%.1f°", error);

        telemetry.addData("─── LIMELIGHT ───", "");
        telemetry.addData("Target Visible", targetVisible ? "YES" : "NO");
        telemetry.addData("tx (offset)", "%.2f°", tx);
        telemetry.addData("Deadband", "±%.1f°", CAMERA_DEADBAND);

        telemetry.addData("─── LIMITS ───", "");
        telemetry.addData("Left Limit", "%.0f°", LEFT_LIMIT);
        telemetry.addData("Right Limit", "%.0f°", RIGHT_LIMIT);
        telemetry.addData("Unwind Threshold", "±%.0f°", UNWIND_THRESHOLD);

        // Show when near unwind zone
        if (currentAngle > (RIGHT_LIMIT - UNWIND_THRESHOLD) && !isUnwinding) {
            telemetry.addData("UNWIND ZONE", "Near right limit");
        }
        if (currentAngle < (LEFT_LIMIT + UNWIND_THRESHOLD) && !isUnwinding) {
            telemetry.addData("UNWIND ZONE", "Near left limit");
        }
    }
}