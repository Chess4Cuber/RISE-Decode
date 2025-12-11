package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.otherMechanisms.slidesArmSystem.OldSlidesStates;
import org.firstinspires.ftc.teamcode.mechanisms.otherMechanisms.slidesArmSystem.OptimusSlides;

public class RadahnTurretSystemManual {
    Gamepad gamepad1;
    Telemetry telemetry;

    RadahnTurretManual turret;

    PID_Controller pid;

    TurretStatesManual turretStateManual;

    boolean lastToggle = false;
    boolean lastToggleB = false;

    public RadahnTurretSystemManual(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        turret = new RadahnTurretManual(gamepad1, telemetry, hardwareMap);


        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }





}
