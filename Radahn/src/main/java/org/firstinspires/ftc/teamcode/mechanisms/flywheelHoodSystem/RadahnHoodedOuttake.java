package org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem;

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

        motors[0].reset();
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    public double getRPMMotor(){
        return motors[0].getVelocityRPM();
    }

    public void setVelocityRPM(double rpm) {
        double ticksPerRevolution = motors[0].TICKS_PER_REV;
        double ticksPerSecond = (rpm * ticksPerRevolution) / 60.0;
        motors[0].dcMotorEx.setVelocity(ticksPerSecond);
    }

}
