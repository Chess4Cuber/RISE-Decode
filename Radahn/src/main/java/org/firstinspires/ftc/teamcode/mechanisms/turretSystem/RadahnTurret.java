package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides;

public class RadahnTurret extends PulleySlides {

    public RadahnTurret(int motorCount, String[] motorNames, double pulleyRadius, double cpr, PID_Controller pid, HardwareMap hardwareMap) {
        super(motorCount, motorNames, pulleyRadius, cpr, RiggingMethod.CONTINUOUS, 1, 0, pid, hardwareMap);
    }

    public void setTargetAngle(double angleRadians){
        double targetExtension = angleRadians * pulleyRadius;
        setExtension(targetExtension);
    }

    public double getCurrentAngle(){
        return getExtension() / pulleyRadius;
    }
}
