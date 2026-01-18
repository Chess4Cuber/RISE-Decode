package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides;

public class RadahnTurret extends PulleySlides {

    public static final double TICKS_PER_REV = 537.6 * 3.0;

    public RadahnTurret(Gamepad gamepad, HardwareMap hardwareMap) {
        super(1, new String[]{"turretMotor"}, 1, TICKS_PER_REV,
                RiggingMethod.CONTINUOUS, 1, 0.0,
                new PID_Controller(0.02, 0.001, 0.8, 0.0),
                hardwareMap);

        slidesPID.tolerance = 0.5;
        motors[0].resetTurret();
        motors[0].setDirectionReverse();
    }

    public double getTurretAngle() {
        return motors[0].getCurrPosDegrees();
    }

    public void setTurretPower(double power) {
        setPower(power);
    }
}
