package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides;

public class RadahnTurret extends PulleySlides {

    public RadahnTurret(HardwareMap hardwareMap) {
        super(1, new String[]{"turretMotor"}, 1.0, 435, RiggingMethod.CONTINUOUS, 1, 0.0, new PID_Controller(0.005, 0.01, 0.7, 0.0001), hardwareMap);
    }

    public void setTargetAngle(double angleRadians) {
        double targetExtension = angleRadians * pulleyRadius;
        setExtension(targetExtension);
    }

    public double getCurrentAngle() {
        return getExtension() / pulleyRadius;
    }

}