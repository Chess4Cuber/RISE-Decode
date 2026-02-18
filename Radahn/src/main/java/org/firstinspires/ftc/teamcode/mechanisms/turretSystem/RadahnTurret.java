package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurret {

    public Motor turretMotor;
    public PID_Controller turretPID;

    public static final double GOBILDA_CPR = 537.7;
    public static final double GEAR_RATIO = 3.0;
    public static final double TURRET_CPR = GOBILDA_CPR * GEAR_RATIO;

    private static final double KP = 0.02;
    private static final double KD = 0.001;
    private static final double KI = 0.0;

    private static final double ANGLE_TOLERANCE = 2.0;

    public RadahnTurret(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        turretMotor = new Motor("turretMotor", TURRET_CPR, hardwareMap);

        turretMotor.resetTurret();

        turretMotor.setBreakMode();

        turretPID = new PID_Controller(KP, KD, 0.7, KI);
        turretPID.tolerance = ANGLE_TOLERANCE;
    }

    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    public double getAngleDegrees() {
        return turretMotor.getCurrPosDegrees();
    }

    public double setTargetAngle(double targetAngle, double maxPower) {
        double currentAngle = getAngleDegrees();

        double pidPower = turretPID.PID_Power(currentAngle, targetAngle);

        double clampedPower = Math.max(-maxPower, Math.min(maxPower, pidPower));

        turretMotor.setPower(clampedPower);

        return clampedPower;
    }

    public boolean isAtTarget(double targetAngle) {
        double error = Math.abs(targetAngle - getAngleDegrees());
        return error < ANGLE_TOLERANCE;
    }

    public double getError(double targetAngle) {
        return targetAngle - getAngleDegrees();
    }

    public void stop() {
        turretMotor.setPower(0);
        turretPID.area = 0;
    }
}