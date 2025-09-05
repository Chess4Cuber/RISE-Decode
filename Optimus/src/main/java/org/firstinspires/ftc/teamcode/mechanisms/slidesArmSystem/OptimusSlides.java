package org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem;

import static org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides.RiggingMethod.CONTINUOUS;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OptimusSlides extends PulleySlides {

    public OptimusSlides(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(2, new String[]{"slidesLeft", "slidesRight"}, 1.5, 387.5, RiggingMethod.CONTINUOUS, 2, 0.2, new PID_Controller(0.05,0,0,0.025), hardwareMap);
    }
}
