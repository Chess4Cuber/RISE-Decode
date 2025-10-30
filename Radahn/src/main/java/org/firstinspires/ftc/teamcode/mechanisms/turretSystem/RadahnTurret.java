package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides;

public class RadahnTurret extends PulleySlides {

    public RadahnTurret(HardwareMap hardwareMap) {
        super(
                1,
                new String[]{"turretMotor"},
                1.0,       // pulley radius
                435,       // counts per revolution
                RiggingMethod.CONTINUOUS,
                1,         // stages
                0.0,       // gravity factor
                new PID_Controller(0.002, 0.07, 0.7, 0.0001),
                hardwareMap
        );

        // Flip motor direction so turret rotates correctly
        motors[0].setDirectionReverse();

        // Reset encoder and put motor in RUN_USING_ENCODER mode for PID control
        motors[0].resetTurret();
    }


    public void setTargetAngle(double angleRadians) {
        double targetExtension = angleRadians * pulleyRadius;
        setExtension(targetExtension);
    }

    public double getCurrentAngle() {
        return getExtension() / pulleyRadius;
    }
}
