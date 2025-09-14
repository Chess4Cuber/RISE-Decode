package org.firstinspires.ftc.teamcode.mechanisms.motorIntake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.PassiveIntake;
import org.firstinspires.ftc.baseCode.hardware.ServoActiveIntake;
import org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorOptimusIntake extends PassiveIntake {
    public MotorOptimusIntake(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(1, new String[]{"flyMotor"}, 387.5, gamepad1, telemetry, hardwareMap);
    }
}

