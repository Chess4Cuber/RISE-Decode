package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

public class RadahnTurretSystem {

    private final RadahnTurret turret;
    private final Telemetry telemetry;
    private final PID_Controller pid;

    private final double cameraWidth;
    private final double cameraFOV;

    private static final double CENTER_TOLERANCE_PIXELS = 30;
    private static final double MIN_POWER = 0.03;
    private static final double MAX_POWER = 0.25;
    private static final double SMOOTHING = 0.2;

    private TurretStates turretState;
    private double lastPower = 0;

    public RadahnTurretSystem(HardwareMap hardwareMap, Telemetry telemetry, double pulleyRadius, PID_Controller pid, double cameraWidth, double cameraFOV) {
        turret = new RadahnTurret(hardwareMap);

        this.telemetry = telemetry;
        this.pid = pid;
        this.cameraWidth = cameraWidth;
        this.cameraFOV = cameraFOV;

        turretState = TurretStates.RESTING;
    }

    public void update(List<AprilTagDetection> detections, int targetTagID) {
        double currentAngle = turret.getCurrentAngle();
        double targetAngle = currentAngle;

        boolean foundTarget = false;

        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == targetTagID) {

                    // Use center.x for horizontal position in pixels
                    double offsetPixels = tag.center.x - (cameraWidth / 2.0);

                    if (Math.abs(offsetPixels) > CENTER_TOLERANCE_PIXELS) {
                        double offsetRatio = offsetPixels / (cameraWidth / 2.0);
                        double angleOffset = offsetRatio * (cameraFOV / 2.0);

                        targetAngle = currentAngle + angleOffset;
                        turretState = TurretStates.TRACKING;
                    } else {
                        turretState = TurretStates.RESTING;
                    }

                    foundTarget = true;
                    break;
                }
            }
        }

        if (!foundTarget) {
            turretState = TurretStates.RESTING;
        }

        double power;

        if (turretState == TurretStates.TRACKING) {
            power = pid.PID_Power(currentAngle, targetAngle);
            power = clamp(power, -MAX_POWER, MAX_POWER);

            // Smoothing + deadband
            power = SMOOTHING * power + (1 - SMOOTHING) * lastPower;
            if (Math.abs(power) < MIN_POWER) power = 0;

            turret.setPower(power);
        } else {
            turret.setPower(0);
            power = 0;
        }

        lastPower = power;

        // Telemetry
        telemetry.addData("Turret State", turretState);
        telemetry.addData("Turret Angle (rad)", turret.getCurrentAngle());
        telemetry.addData("Target Angle (rad)", targetAngle);
        telemetry.addData("Power", power);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public void setTurretState(TurretStates state) {
        turretState = state;
    }

    public RadahnTurret getTurret() {
        return turret;
    }
}
