package org.firstinspires.ftc.teamcode.mechanisms.turretManual;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurretSystemManual {
    Gamepad gamepad1;
    Telemetry telemetry;

    RadahnTurretManual turret;

    PID_Controller pid;

    TurretStatesManual turretStateManual;

    boolean lastToggleRB = false;

    boolean lastToggleLB = false;

    public RadahnTurretSystemManual(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        turret = new RadahnTurretManual(gamepad1, telemetry, hardwareMap);
        turretStateManual = TurretStatesManual.MANUAL;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

    }


    public void setPositions(){
        switch (turretStateManual){
            case RESTING:
                break;

            case MANUAL:
                if (gamepad1.right_bumper){
                    turret.setPower(0.75);
                } else if (gamepad1.dpad_down){
                    turret.setPower(-0.75);
                } else {
                    turret.setPower(0);
                }

                break;
        }
    }
    public void controllerInput(){
        switch (turretStateManual) {
            case RESTING:
                if ((gamepad1.right_bumper != lastToggleRB) && gamepad1.right_bumper) {
                    turretStateManual = turretStateManual.MANUAL;
                }
                if((gamepad1.left_bumper != lastToggleLB) && gamepad1.left_bumper){
                    turretStateManual = turretStateManual.MANUAL;
                }
                break;
        }
        lastToggleRB = gamepad1.right_bumper;
        lastToggleLB = gamepad1.left_bumper;
    }




}
