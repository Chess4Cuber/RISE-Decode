package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

public class RadahnTurretSystem {

    RadahnTurret turret;
    Telemetry telemetry;
    TurretStates turretState;

    private final PID_Controller pid;

    private final double cameraWidth;
    private final double cameraFOV; // in radians

    private static final double CENTER_TOLERANCE_PIXELS = 10; // tolerance for horizontal offset

    public RadahnTurretSystem(HardwareMap hardwareMap, Telemetry telemetry, double pulleyRadius, PID_Controller pid, double cameraWidth, double cameraFOV) {
        turret = new RadahnTurret(1, new String[]{"turretMotor"}, pulleyRadius, 435, pid, hardwareMap);

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
                    // Calculate horizontal offset from camera center
                    double offsetPixels = tag.pose.x; // positive = right, negative = left
                    if (Math.abs(offsetPixels) > CENTER_TOLERANCE_PIXELS) {
                        double offsetRatio = offsetPixels / (cameraWidth / 2.0); // -1 to 1
                        double angleOffset = offsetRatio * (cameraFOV / 2.0); // radians

                        targetAngle = currentAngle + angleOffset;
                        turretState = TurretStates.TRACKING;
                    } else {
                        turretState = TurretStates.RESTING; // within tolerance
                    }
                    foundTarget = true;
                    break;
                }
            }
        }

        if (!foundTarget) {
            turretState = TurretStates.RESTING;
        }

        // Apply PID only if outside tolerance
        if (turretState == TurretStates.TRACKING) {
            double power = pid.PID_Power(currentAngle, targetAngle);
            turret.setTargetAngle(currentAngle + power);
        } else {
            turret.setPower(0);
        }

        telemetry.addData("Turret Angle (rad)", turret.getCurrentAngle());
        telemetry.addData("Turret State", turretState);
    }

    public void setTurretState(TurretStates state){
        turretState = state;
    }

    public RadahnTurret getTurret() {
        return turret;
    }
}
