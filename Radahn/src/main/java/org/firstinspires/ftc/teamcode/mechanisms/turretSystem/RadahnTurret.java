package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.baseCode.hardware.extension.PulleySlides;
import org.firstinspires.ftc.baseCode.control.PID_Controller;

public class RadahnTurret extends PulleySlides {

    public Limelight3A limelight;
    public PID_Controller turretPID;

    public RadahnTurret(HardwareMap hardwareMap) {
        // Single motor, CONTINUOUS pulley, 435 RPM
        super(1, new String[]{"turretMotor"}, 1, 435, RiggingMethod.CONTINUOUS, 1, 0,
                new PID_Controller(0.05, 0.01, 0.1), hardwareMap);

        // Limelight initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        turretPID = new PID_Controller(0.05, 0.01, 0.1);
        turretPID.tolerance = 0.5;
    }

    public double getTurretAngle() {
        return getExtension() * (180 / Math.PI);
    }

    public void setTurretAngle(double targetAngle) {
        double power = turretPID.PID_Power(getTurretAngle(), targetAngle);
        setPower(power);
    }

    public boolean hasTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public double getTargetOffset() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }
}
