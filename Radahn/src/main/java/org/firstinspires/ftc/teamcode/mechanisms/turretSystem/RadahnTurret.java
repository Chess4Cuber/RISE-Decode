package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.baseCode.hardware.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurret {

    public Motor turretMotor;

    // 537.7 CPR motor * 3:1 gear ratio
    public static final double TURRET_CPR = 537.7 * 3;

    public RadahnTurret(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        turretMotor = new Motor("turretMotor", TURRET_CPR, hardwareMap);
        turretMotor.resetTurret();
        turretMotor.setBreakMode();
    }

    public void setPower(double power){
        turretMotor.setPower(power);
    }

    public double getAngleDegrees(){
        return turretMotor.getCurrPosDegrees();
    }
}
