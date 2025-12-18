package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.turretSystem.RadahnTurretSystem;


@TeleOp
public class TurretTesting extends LinearOpMode {

    RadahnTurretSystem turret;
    RadahnChassis chassis;

    @Override
    public void runOpMode() {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        turret = new RadahnTurretSystem(gamepad1, telemetry, hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Waiting for start");
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            turret.controllerInput();
            turret.setPositions();

            chassis.robotCentricDrive();
            chassis.updatePose();
        }
    }
}
