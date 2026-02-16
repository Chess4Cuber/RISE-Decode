package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurretSystem {

    Gamepad gamepad1;
    Telemetry telemetry;
    RadahnTurret turret;

    public TurretStates turretState;

    private static final double LEFT_LIMIT = -160.0;   // Maximum left rotation
    private static final double RIGHT_LIMIT = 160.0;   // Maximum right rotation

    private static final double MANUAL_STEP = 5.0;

    // how aggressively to correct for tx
    private static final double AUTO_TRACKING_GAIN = 0.4;

    private static final double MAX_POWER = 0.5;

    private double targetAngle = 0;

    private double tx = 0;
    private boolean targetVisible = false;

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
            }
        }

        else if ((gamepad1.left_bumper != lastLB) && gamepad1.left_bumper) {
            turretState = TurretStates.MANUAL;
            targetAngle -= MANUAL_STEP;

            targetAngle = clamp(targetAngle, LEFT_LIMIT, RIGHT_LIMIT);
        }

        else if ((gamepad1.right_bumper != lastRB) && gamepad1.right_bumper) {
            turretState = TurretStates.MANUAL;
            targetAngle += MANUAL_STEP;
            // Clamp to limits
            targetAngle = clamp(targetAngle, LEFT_LIMIT, RIGHT_LIMIT);
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
                break;

            case MANUAL:
                appliedPower = turret.setTargetAngle(targetAngle, MAX_POWER);
                break;

            case AUTO_AIM:

                if (targetVisible) {

                    targetAngle += tx * AUTO_TRACKING_GAIN;

                    // Option 2: Direct correction (more aggressive)
                    // targetAngle = currentAngle + tx;

                    targetAngle = clamp(targetAngle, LEFT_LIMIT, RIGHT_LIMIT);
                }

                appliedPower = turret.setTargetAngle(targetAngle, MAX_POWER);
                break;
        }


        applySoftLimits(appliedPower, currentAngle);
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
        return turret.isAtTarget(targetAngle);
    }


    private void displayTelemetry() {
        double currentAngle = turret.getAngleDegrees();
        double error = turret.getError(targetAngle);
        boolean atTarget = turret.isAtTarget(targetAngle);

        telemetry.addData("─── TURRET STATUS ───", "");
        telemetry.addData("Mode", turretState);
        telemetry.addData("At Target", atTarget ? "YES" : "NO");

        telemetry.addData("─── ANGLES ───", "");
        telemetry.addData("Current Angle", "%.1f°", currentAngle);
        telemetry.addData("Target Angle", "%.1f°", targetAngle);
        telemetry.addData("Error", "%.1f°", error);

        telemetry.addData("─── LIMELIGHT ───", "");
        telemetry.addData("Target Visible", targetVisible ? "YES" : "NO");
        telemetry.addData("tx (offset)", "%.2f°", tx);

        telemetry.addData("─── LIMITS ───", "");
        telemetry.addData("Left Limit", "%.0f°", LEFT_LIMIT);
        telemetry.addData("Right Limit", "%.0f°", RIGHT_LIMIT);

        if (currentAngle < LEFT_LIMIT + 20) {
            telemetry.addData("WARNING", "Approaching LEFT limit!");
        }
        if (currentAngle > RIGHT_LIMIT - 20) {
            telemetry.addData("WARNING", "Approaching RIGHT limit!");
        }
    }
}