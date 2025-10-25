package org.firstinspires.ftc.teamcode.mechanisms.simpleMotorOuttakeSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.PassiveIntake;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnMotorOuttake extends PassiveIntake {
    public RadahnMotorOuttake(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(1, new String[]{"flyMotor"}, 387.5, gamepad1, telemetry, hardwareMap);
    }
}

