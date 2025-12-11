package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnPusher;
import org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.RadahnHoodedOuttake;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.simpleMotorOuttakeSystem.RadahnMotorOuttakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.turretManual.RadahnTurretSystemManual;

@TeleOp
public class RadahnSimpleTeleOp extends LinearOpMode {
    RadahnHoodedOuttake hood;
    RadahnMotorOuttakeSystem simpleOuttake;
    RadahnTurretSystemManual turret;
    RadahnChassis chassis;
    RadahnMotorIntakeSystem intake;
    RadahnPusher pusher;

    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hood = new RadahnHoodedOuttake(gamepad1, telemetry, hardwareMap);
        simpleOuttake = new RadahnMotorOuttakeSystem(gamepad1, telemetry, hardwareMap);
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        intake = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);
        pusher = new RadahnPusher(gamepad1, hardwareMap);
        turret = new RadahnTurretSystemManual(gamepad1, telemetry, hardwareMap);


        while (opModeInInit()) {
            pusher.openClaw();
            hood.setHoodPosition(-.3);

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()) {

            chassis.robotCentricDrive();
            chassis.updatePose();

            simpleOuttake.controllerInput();
            simpleOuttake.setPositions();

            intake.controllerInput();
            intake.setPositions();

            turret.controllerInput();
            turret.setPositions();

            pusher.toggleClaw();

            telemetry.addData("Loop Time", runtime.seconds() - previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }

    }
}
