package org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.baseCode.hardware.PassiveIntake;
import org.firstinspires.ftc.baseCode.hardware.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnHoodedOuttake extends PassiveIntake {

    Servo hoodServo;

    public RadahnHoodedOuttake(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(2, new String[]{"flyMotor", "flyMotor2"}, 28, gamepad1, telemetry, hardwareMap);

        for (Motor motor : motors) {
            motor.encode();
        }

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }

    public void setHoodPosition(double pos) {
        hoodServo.setPosition(pos);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    // --- Flywheel Control --
    public void setVelocityRPM(double rpm) {
        for (Motor motor : motors) {
            motor.setVelocityRPM(rpm);
        }
    }

    public double getRPMMotor() {
        double sum = 0;
        for (Motor motor : motors) {
            sum += motor.getVelocityRPM();
        }
        return sum / motors.length;
    }
}
