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

    // Camera geometry
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

        // ---- Limelight Init ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(activePipeline);
        limelight.start();   // <<< REQUIRED FOR FTC SDK DATA STREAM

        while (opModeInInit()) {

            pusher.openClaw();
            hoodedOuttakeSystem.setMotorOuttakeState(
                    org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.TurretHoodStates.RESTING
            );

            // Pipeline toggle
            if ((gamepad1.y != lastToggleY) && gamepad1.y) {
                if (activePipeline == 0) {
                    activePipeline = 1;
                } else {
                    activePipeline = 0;
                }
                limelight.pipelineSwitch(activePipeline);
            }
            lastToggleY = gamepad1.y;

            telemetry.addLine("Waiting For Start");
            telemetry.addData("Active Pipeline", activePipeline);
            telemetry.update();
        }

        while (opModeIsActive()) {

            chassis.robotCentricDrive();
            chassis.updatePose();

            double tagDistanceInches = 0;
            double tx = 0;
            boolean tagVisible = false;

            LLResult result = limelight.getLatestResult();

            // ---- Limelight Read ----
            if (result != null) {
                if (result.isValid()) {
                    tagVisible = true;

                    double tyRadians = Math.toRadians(result.getTy());
                    tagDistanceInches = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                            Math.tan(CAMERA_ANGLE + tyRadians);

                    tx = result.getTx();
                }
            }

            // ---- Hood + Flywheel ----
            hoodedOuttakeSystem.updateDistance(tagDistanceInches);
            hoodedOuttakeSystem.update();

            // ---- Intake ----
            intake.controllerInput();
            intake.setPositions();

            // ---- Turret ----
            turret.updateTargetAngle(tx);
            turret.controllerInput();
            turret.setPositions();
            turret.setTelemetry();

            // ---- Pusher ----
            pusher.toggleClaw();

            // ---- Telemetry ----
            telemetry.addData("AprilTag Visible", tagVisible ? "YES" : "NO");
            //telemetry.addData("Limelight Pipeline", limelight.getPipelineIndex());
            telemetry.addData("tx", tx);
            telemetry.addData("Loop Time", runtime.seconds() - previousTime);

            // Extra debug
            telemetry.addData("LL Result Null", result == null);
            if(result != null){
                telemetry.addData("LL Valid", result.isValid());
            }

            telemetry.update();
            previousTime = runtime.seconds();
        }
    }
}
