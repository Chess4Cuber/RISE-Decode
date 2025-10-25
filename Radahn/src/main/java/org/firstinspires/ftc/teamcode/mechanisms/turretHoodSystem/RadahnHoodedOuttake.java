package org.firstinspires.ftc.teamcode.mechanisms.turretHoodSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.baseCode.hardware.PassiveIntake;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnHoodedOuttake extends PassiveIntake {

    public Servo hoodServo;

    public RadahnHoodedOuttake(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(1, new String[]{"flyMotor"}, 387.5, gamepad1, telemetry, hardwareMap);
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }
}
