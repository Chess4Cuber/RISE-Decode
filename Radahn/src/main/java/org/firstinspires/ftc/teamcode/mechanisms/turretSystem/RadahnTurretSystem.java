package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurretSystem {

    Gamepad gamepad;
    Telemetry telemetry;
    RadahnTurret turret;

    public TurretStates turretState;

    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    double targetAngle = 0;

    // Step size per press (degrees)
    public static final double STEP_ANGLE = 12;

    public RadahnTurretSystem(Gamepad gamepad, Telemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        turret = new RadahnTurret(gamepad, hardwareMap);

        turretState = TurretStates.TRACKING;
        targetAngle = turret.getTurretAngle();
    }

    public void updateTargetAngle(double tx) {
        if (turretState == TurretStates.TRACKING) {
            targetAngle = turret.getTurretAngle() + tx;
            targetAngle = turret.clampAngle(targetAngle);
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
                    targetAngle = turret.clampAngle(targetAngle);
                }

                if ((rightBumper != lastRightBumper) && rightBumper) {
                    turretState = TurretStates.MANUAL;
                    targetAngle = turret.getTurretAngle() + STEP_ANGLE;
                    targetAngle = turret.clampAngle(targetAngle);
                }
                break;

            case MANUAL:
                if (leftBumper && rightBumper) {
                    turretState = TurretStates.TRACKING;
                }

                if ((leftBumper != lastLeftBumper) && leftBumper) {
                    targetAngle -= STEP_ANGLE;
                    targetAngle = turret.clampAngle(targetAngle);
                }

                if ((rightBumper != lastRightBumper) && rightBumper) {
                    targetAngle += STEP_ANGLE;
                    targetAngle = turret.clampAngle(targetAngle);
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
        telemetry.addData("PID P", turret.slidesPID.P);
        telemetry.addData("PID I", turret.slidesPID.I);
        telemetry.addData("PID D", turret.slidesPID.D);
    }
}
