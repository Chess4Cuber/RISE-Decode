package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurretManual extends PulleySlides {
    public RadahnTurretManual(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(1, new String[]{"turretMotor"}, 1.0, 435, PulleySlides.RiggingMethod.CONTINUOUS, 1, 0.0, new PID_Controller(0.002, 0.07, 0.7, 0.0001), hardwareMap);

        // Flip motor direction so turret rotates correctly
        motors[0].setDirectionForward();

        // Reset encoder and put motor in RUN_USING_ENCODER mode for PID control
        motors[0].resetTurret();
    }
}
