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
    double maxFlywheelRPM = 6000;

    public RadahnHoodedOuttake(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(2, new String[]{"flyMotor", "flyMotor2"}, 28, gamepad1, telemetry, hardwareMap);
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        flywheelPID = new PID_Controller[motors.length];
        for (int i = 0; i < flywheelPID.length; i++) {
            flywheelPID[i] = new PID_Controller(0.001, 0.00025, 0.7, 0.0002);
            flywheelPID[i].tolerance = 25;
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setHoodPosition(double pos) {
        hoodServo.setPosition(pos);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    // --- Flywheel Control using POWER ---
    public void setFlywheelPower(double power) {
        for (int i = 0; i < motors.length; i++) {
            Motor motor = motors[i];

            if (Math.abs(power) < 0.01) {
                flywheelPID[i] = new PID_Controller(0.001, 0.00025, 0.7, 0.0002);
                flywheelPID[i].tolerance = 25;
                motor.setPower(0);
                continue;
            }

            double targetRPM = Math.abs(power) * maxFlywheelRPM;
            double currentRPM = Math.abs(motor.getVelocityRPM());

            double pidOutput = flywheelPID[i].PID_Power(currentRPM, targetRPM);
            double commandedPower = Math.abs(power) + pidOutput;
            double signedPower = Math.signum(power) * Math.max(0, Math.min(1, commandedPower));

            motor.setPower(signedPower);
        }
    }

    // Optional telemetry if one motor still has encoder
    public double getRPMMotor() {
        return motors[0].getVelocityRPM();
    }
}