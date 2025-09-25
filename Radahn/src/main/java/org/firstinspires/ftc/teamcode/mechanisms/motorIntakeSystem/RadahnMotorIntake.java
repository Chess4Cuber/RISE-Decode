package org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.PassiveIntake;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnMotorIntake extends PassiveIntake {
    public RadahnMotorIntake(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(1, new String[]{"intakeMotor"}, 387.5, gamepad1, telemetry, hardwareMap);
    }
}

