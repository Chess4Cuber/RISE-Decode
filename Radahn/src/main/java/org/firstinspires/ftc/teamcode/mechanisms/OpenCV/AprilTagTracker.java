package org.firstinspires.ftc.teamcode.mechanisms.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.Motor;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

/**
 * AprilTagTracker controls a pan motor to center an AprilTag horizontally.
 * Tuned for a 312 RPM GoBilda motor, with direction toggle.
 */
public class AprilTagTracker extends Motor {

    private final PID_Controller pid;
    private final double frameCenterX;

    private double tolerancePixels = 15; // deadband around center
    private double maxPower = 0.12;       // max motor power
    private double lastPower = 0;
    private double smoothingFactor = 0.25; // low-pass smoothing
    private double stopRangePixels = 20; // motor stops if tag is within ±20 pixels of center

    private final ElapsedTime runtime = new ElapsedTime();

    private int directionMultiplier = 1; // 1 = normal, -1 = reversed

    public AprilTagTracker(String motorName, double cpr, HardwareMap hwmap, double frameWidth) {
        super(motorName, cpr, hwmap);

        this.frameCenterX = frameWidth / 2.0;

        // PID tuned for slower motor
        this.pid = new PID_Controller(0.001, 0.04, 0.7, 0.0001);
        this.pid.tolerance = tolerancePixels;

        // Brake motor when zero power
        dcMotorEx.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Track the first detected tag and move motor to center it.
     * Returns the current applied motor power.
     */
    public double trackTag(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) {
            safeStop();
            return 0;
        }

        AprilTagDetection tag = detections.get(0);
        double currX = tag.center.x;
        double error = frameCenterX - currX;

        // Stop motor if tag is within stop range
        if (Math.abs(error) <= stopRangePixels) {
            safeStop();
            return 0;
        }

        // Compute PID output and apply direction multiplier
        double rawPower = pid.PID_PowerBasic(currX, frameCenterX) * directionMultiplier;

        // Clamp power
        rawPower = clamp(rawPower, -maxPower, maxPower);

        // Low-pass smoothing
        double smoothedPower = smoothingFactor * rawPower + (1 - smoothingFactor) * lastPower;

        // Small deadzone
        if (Math.abs(smoothedPower) < 0.03) smoothedPower = 0;

        // Apply power
        dcMotorEx.setPower(smoothedPower);
        lastPower = smoothedPower;

        return smoothedPower;
    }

    /** Immediately stops the motor */
    public void stopTracking() {
        safeStop();
    }

    private void safeStop() {
        try {
            dcMotorEx.setPower(0);
        } catch (Exception ignored) {}
        lastPower = 0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ---- Direction toggle ----
    public void setDirectionNormal() {
        directionMultiplier = 1;
    }

    public void setDirectionReversed() {
        directionMultiplier = -1;
    }

    // ---- Setters for tuning ----
    public void setTolerancePixels(double tolerancePixels) {
        this.tolerancePixels = tolerancePixels;
        this.pid.tolerance = tolerancePixels;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setPID(double kp, double kd, double ki) {
        pid.setPIDCoefficients(kp, kd, 0, ki);
    }

    public void setSmoothingFactor(double smoothingFactor) {
        this.smoothingFactor = Math.max(0, Math.min(1, smoothingFactor));
    }

    // ---- Getters for telemetry/debug ----
    public double getLastPower() { return lastPower; }
    public double getFrameCenterX() { return frameCenterX; }

    public void setStopRangePixels(double range) {
        this.stopRangePixels = range;
    }

}
