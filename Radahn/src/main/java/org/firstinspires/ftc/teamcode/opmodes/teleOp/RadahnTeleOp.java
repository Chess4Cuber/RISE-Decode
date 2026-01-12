package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnPusher;
import org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.RadahnHoodedOuttakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.turretSystem.RadahnTurretSystem;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp
public class RadahnTeleOp extends LinearOpMode {

    RadahnHoodedOuttakeSystem hoodedOuttakeSystem;
    RadahnTurretSystem turret;
    RadahnChassis chassis;
    RadahnMotorIntakeSystem intake;
    RadahnPusher pusher;

    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;

    Limelight3A limelight;

    final double CAMERA_HEIGHT = 10.0;
    final double TARGET_HEIGHT = 24.0;
    final double CAMERA_ANGLE = Math.toRadians(30.0);

    int activePipeline = 0;
    boolean lastToggleY = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hoodedOuttakeSystem = new RadahnHoodedOuttakeSystem(gamepad1, telemetry, hardwareMap);
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        intake = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);
        pusher = new RadahnPusher(gamepad1, hardwareMap);
        turret = new RadahnTurretSystem(gamepad1, telemetry, hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(activePipeline);

        while (opModeInInit()) {
            pusher.openClaw();
            hoodedOuttakeSystem.setMotorOuttakeState(
                    org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.TurretHoodStates.RESTING
            );

            if ((gamepad1.y != lastToggleY) && gamepad1.y) {
                activePipeline = (activePipeline == 0) ? 1 : 0;
                limelight.pipelineSwitch(activePipeline);
            }
            lastToggleY = gamepad1.y;

            telemetry.addLine("Waiting For Start");
            telemetry.addData("Active Pipeline", activePipeline == 0 ? "Blue Goal" : "Red Goal");
            telemetry.update();
        }

        while (opModeIsActive()) {

            chassis.robotCentricDrive();
            chassis.updatePose();

            double tagDistanceInches = 0;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tyRadians = Math.toRadians(result.getTy());
                tagDistanceInches = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(CAMERA_ANGLE + tyRadians);

                double tx = result.getTx();
                turret.updateTargetAngle(tx);
            }

            hoodedOuttakeSystem.updateDistance(tagDistanceInches);
            hoodedOuttakeSystem.update();

            intake.controllerInput();
            intake.setPositions();

            turret.controllerInput();
            turret.setPositions();

            pusher.toggleClaw();

            turret.setTelemetry();
            telemetry.addData("Loop Time", runtime.seconds() - previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
