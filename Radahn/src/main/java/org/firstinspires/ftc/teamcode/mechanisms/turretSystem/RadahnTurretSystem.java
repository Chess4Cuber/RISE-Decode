package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurretSystem {

    Gamepad gamepad;
    Telemetry telemetry;
    RadahnTurret turret;

    public TurretStates turretState;

    // Toggle tracking buttons
    boolean lastToggleA = false;
    boolean lastToggleY = false;

    double targetAngle = 0;

    public RadahnTurretSystem(Gamepad gamepad, Telemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        turret = new RadahnTurret(gamepad, hardwareMap);
        turretState = TurretStates.IDLE;
    }

    public void updateTargetAngle(double tx) {
        this.targetAngle = getCurrentAngle() + tx;
    }

    public double getCurrentAngle() {
        return turret.getTurretAngle();
    }

    public void controllerInput() {
        if ((gamepad.a != lastToggleA) && gamepad.a) {
            turretState = TurretStates.TRACKING;
        }

        if ((gamepad.y != lastToggleY) && gamepad.y) {
            turretState = TurretStates.IDLE;
        }

        lastToggleA = gamepad.a;
        lastToggleY = gamepad.y;
    }

    public void setPositions() {
        switch (turretState) {

            case IDLE:
                turret.setTurretPower(0);
                break;

            case TRACKING:
                turret.setExtension(targetAngle);
                break;
        }
    }

    public void setTelemetry() {
        telemetry.addData("Turret Angle", getCurrentAngle());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Turret State", turretState);
        telemetry.addData("PID P", turret.slidesPID.P);
        telemetry.addData("PID I", turret.slidesPID.I);
        telemetry.addData("PID D", turret.slidesPID.D);
    }
}
