package org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.PassiveIntake;
import org.firstinspires.ftc.baseCode.hardware.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnHoodedOuttake extends PassiveIntake {

    Servo hoodServo;
    PID_Controller[] flywheelPID;

    double maxFlywheelRPM = 6000.0;

    double kF = 0.000167;

    boolean[] atTargetSpeed;
    double[] targetRPM;

    public RadahnHoodedOuttake(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {

        super(2, new String[]{"flyMotor", "flyMotor2"}, 537.7, gamepad1, telemetry, hardwareMap);
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        flywheelPID = new PID_Controller[motors.length];
        atTargetSpeed = new boolean[motors.length];
        targetRPM = new double[motors.length];

        for (int i = 0; i < flywheelPID.length; i++) {

            flywheelPID[i] = new PID_Controller(0.0003, 0.00008, 0.7, 0.00001);

            // RPM tolerance before integral windup is reset
            flywheelPID[i].tolerance = 50;

            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motors[i].setBreakMode();

            atTargetSpeed[i] = false;
            targetRPM[i] = 0;
        }
    }

    public void setHoodPosition(double pos) {
        hoodServo.setPosition(pos);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }


    public void setFlywheelPower(double power) {
        for (int i = 0; i < motors.length; i++) {
            Motor motor = motors[i];

            if (Math.abs(power) < 0.01) {
                motor.setPower(0);
                flywheelPID[i].area = 0;
                atTargetSpeed[i] = false;
                targetRPM[i] = 0;
                continue;
            }

            // Convert power (0-1) to target RPM based on motor max speed
            double targetSpeed = Math.abs(power) * maxFlywheelRPM;
            targetRPM[i] = targetSpeed;

            // Get current motor speed in RPM
            double currentSpeed = Math.abs(motor.getVelocityRPM());

            // Calculate error for status tracking
            double speedError = Math.abs(targetSpeed - currentSpeed);
            atTargetSpeed[i] = speedError < 100; // Within 100 RPM = at speed

            double feedforward = targetSpeed * kF;

            double pidCorrection = flywheelPID[i].PID_Power(currentSpeed, targetSpeed);

            double totalPower = feedforward + pidCorrection;
            double signedPower = Math.signum(power) * Math.max(0.0, Math.min(1.0, totalPower));

            motor.setPower(signedPower);
        }
    }

    public double getRPMMotor() {
        return motors[0].getVelocityRPM();
    }

    public double getTargetRPM(double power) {
        return Math.abs(power) * maxFlywheelRPM;
    }

    public boolean isAtTargetSpeed() {
        for (boolean atSpeed : atTargetSpeed) {
            if (!atSpeed) return false;
        }
        return true;
    }

    public double getAverageRPMError() {
        double totalError = 0;
        for (int i = 0; i < motors.length; i++) {
            totalError += Math.abs(targetRPM[i] - Math.abs(motors[i].getVelocityRPM()));
        }
        return totalError / motors.length;
    }
}