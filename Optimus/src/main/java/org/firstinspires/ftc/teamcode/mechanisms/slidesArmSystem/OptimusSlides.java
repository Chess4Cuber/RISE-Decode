package org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OptimusSlides extends PulleySlides {

    public OptimusSlides(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(2, new String[]{"slidesLeft", "slidesRight"}, 1.5, 387.5,RiggingMethod.CONTINUOUS, 2, 0, hardwareMap);
    }
}
