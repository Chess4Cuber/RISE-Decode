package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurretSystem {

    Gamepad gamepad1;
    Telemetry telemetry;

    RadahnTurret turret;
    TurretStates turretState;

    boolean lastRightBumper = false;
    boolean lastLeftBumper = false;

    double manualIncrement = 15; // degrees per bumper press
    double targetAngle;

    public RadahnTurretSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        turret = new RadahnTurret(hardwareMap);

        // PID tuned for snap-to-target
        turret.turretPID.setPIDCoefficients(0.15, 0.05, 0.1);
        turret.turretPID.tolerance = 0.5;

        turretState = TurretStates.TRACKING;
        targetAngle = turret.getTurretAngle();
    }

    public void setPositions() {
        switch (turretState) {
            case TRACKING:
                if (turret.hasTarget()) {
                    double offset = turret.getTargetOffset();
                    targetAngle = turret.getTurretAngle() + offset;
                }

                double power = turret.turretPID.PID_Power(turret.getTurretAngle(), targetAngle);

                // Minimum power threshold to overcome static friction
                if (Math.abs(power) < 0.2) {
                    power = Math.signum(power) * 0.2;
                }

                turret.setPower(power);
                break;

            case MANUAL_LEFT:
            case MANUAL_RIGHT:
            case IDLE:
                turret.setTurretAngle(targetAngle);
                break;
        }

        setTelemetry();
    }

    public void controllerInput() {

        switch (turretState) {
            case TRACKING:
            case IDLE:

                if ((gamepad1.right_bumper != lastRightBumper) && gamepad1.right_bumper) {
                    targetAngle += manualIncrement;
                    turretState = TurretStates.MANUAL_RIGHT;
                } else if ((gamepad1.left_bumper != lastLeftBumper) && gamepad1.left_bumper) {
                    targetAngle -= manualIncrement;
                    turretState = TurretStates.MANUAL_LEFT;
                } else if (turret.hasTarget()) {
                    turretState = TurretStates.TRACKING;
                } else {
                    turretState = TurretStates.IDLE;
                }
                break;

            case MANUAL_LEFT:
                // Stay manual until both bumpers are pressed
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    turretState = turret.hasTarget() ? TurretStates.TRACKING : TurretStates.IDLE;
                } else if ((gamepad1.left_bumper != lastLeftBumper) && gamepad1.left_bumper) {
                    targetAngle -= manualIncrement;
                } else if ((gamepad1.right_bumper != lastRightBumper) && gamepad1.right_bumper) {
                    targetAngle += manualIncrement;
                }
                break;

            case MANUAL_RIGHT:
                // Stay manual until both bumpers are pressed
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    turretState = turret.hasTarget() ? TurretStates.TRACKING : TurretStates.IDLE;
                } else if ((gamepad1.right_bumper != lastRightBumper) && gamepad1.right_bumper) {
                    targetAngle += manualIncrement;
                } else if ((gamepad1.left_bumper != lastLeftBumper) && gamepad1.left_bumper) {
                    targetAngle -= manualIncrement;
                }
                break;
        }

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;
    }

    public void setTurretState(TurretStates state) {
        turretState = state;
    }

    public void setTelemetry() {
        telemetry.addData("Turret State", turretState);
        telemetry.addData("Turret Angle", turret.getTurretAngle());
        telemetry.addData("Target Visible", turret.hasTarget());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.update();
    }
}
