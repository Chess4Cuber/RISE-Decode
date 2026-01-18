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

    // Camera geometry constants
    final double CAMERA_HEIGHT = 10.0;     // inches
    final double TARGET_HEIGHT = 24.0;     // inches
    final double CAMERA_ANGLE = Math.toRadians(30.0); // radians

    int activePipeline = 0; // 0 = Blue Tag Pipeline, 1 = Red Tag Pipeline
    boolean lastToggleY = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hoodedOuttakeSystem = new RadahnHoodedOuttakeSystem(gamepad1, telemetry, hardwareMap);
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        intake = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);
        pusher = new RadahnPusher(gamepad1, hardwareMap);
        turret = new RadahnTurretSystem(gamepad1, telemetry, hardwareMap);

        // --- Limelight Hardware ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(activePipeline);

        // ---------------- INIT LOOP ----------------
        while (opModeInInit()) {

            pusher.openClaw();
            hoodedOuttakeSystem.setMotorOuttakeState(
                    org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.TurretHoodStates.RESTING
            );

            // Toggle pipeline with Y button
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
            telemetry.addData("Active Pipeline", activePipeline == 0 ? "Blue Goal" : "Red Goal");
            telemetry.update();
        }

        // ---------------- START CAMERA AFTER START ----------------
        waitForStart();

        limelight.start(); // REQUIRED: start streaming after OpMode starts

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {

            chassis.robotCentricDrive();
            chassis.updatePose();

            double tagDistanceInches = 0;
            double tx = 0;
            boolean tagVisible = false;

            // ---- Limelight Read ----
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                tagVisible = true;

                // Horizontal offset for turret tracking
                tx = result.getTx();

                // Vertical angle for distance calculation
                double tyRadians = Math.toRadians(result.getTy());
                tagDistanceInches = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                        Math.tan(CAMERA_ANGLE + tyRadians);
            }

            // --- Update Flywheel + Hood ---
            hoodedOuttakeSystem.updateDistance(tagDistanceInches, tagVisible);
            hoodedOuttakeSystem.update();

            // --- Intake ---
            intake.controllerInput();
            intake.setPositions();

            // --- Turret Auto Tracking ---
//            turret.updateTargetAngle(tx, tagVisible);
//            turret.controllerInput();
//            turret.setPositions();
//            turret.setTelemetry();

            // --- Pusher ---
            pusher.toggleClaw();

            // --- Telemetry ---
            telemetry.addData("AprilTag Visible", tagVisible);
            telemetry.addData("tx", tx);
            telemetry.addData("Distance (in)", tagDistanceInches);
            telemetry.addData("Pipeline", activePipeline);
            telemetry.addData("Loop Time", runtime.seconds() - previousTime);

            telemetry.update();
            previousTime = runtime.seconds();
        }
    }
}
