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

    double manualIncrement = 15.0; // degrees per press
    double targetAngle;

    public RadahnTurretSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        turret = new RadahnTurret(hardwareMap);

        // Let PulleySlides handle PID internally
        turret.turretPID.setPIDCoefficients(0.1, 0.05, 0.1);
        turret.turretPID.tolerance = 0.5;

        turretState = TurretStates.TRACKING;
        targetAngle = turret.getTurretAngle();
    }

    public void controllerInput() {

        boolean rightPressed =
                (gamepad1.right_bumper != lastRightBumper) && gamepad1.right_bumper;
        boolean leftPressed =
                (gamepad1.left_bumper != lastLeftBumper) && gamepad1.left_bumper;

        switch (turretState) {

            case TRACKING:
            case IDLE:
                if (rightPressed) {
                    targetAngle += manualIncrement;
                    turretState = TurretStates.MANUAL_RIGHT;
                } else if (leftPressed) {
                    targetAngle -= manualIncrement;
                    turretState = TurretStates.MANUAL_LEFT;
                }
                break;

            case MANUAL_RIGHT:
            case MANUAL_LEFT:
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    turretState = turret.hasTarget()
                            ? TurretStates.TRACKING
                            : TurretStates.IDLE;
                } else if (rightPressed) {
                    targetAngle += manualIncrement;
                } else if (leftPressed) {
                    targetAngle -= manualIncrement;
                }
                break;
        }

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;
    }

    public void setPositions() {

        double currentAngle = turret.getTurretAngle();

        if (turretState == TurretStates.TRACKING && turret.hasTarget()) {
            targetAngle = currentAngle + turret.getTargetOffset();
        }

        // Convert angle → extension (radians)
        double targetExtension = targetAngle * (Math.PI / 180.0);

        // THIS is the critical fix
        turret.setExtension(targetExtension);

        setTelemetry();
    }

    public void setTurretState(TurretStates state) {
        turretState = state;
    }

    public void setTelemetry() {
        telemetry.addData("Turret State", turretState);
        telemetry.addData("Current Angle", turret.getTurretAngle());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Extension Target", targetAngle * (Math.PI / 180.0));
        telemetry.addData("Has Target", turret.hasTarget());
        telemetry.update();
    }
}
