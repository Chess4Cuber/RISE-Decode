package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.CameraVision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.mechanisms.turretSystem.RadahnTurretSystem;
import org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.RadahnHoodedOuttakeSystem;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "Turret + Hood Combined Testing", group = "Testing")
public class FullOuttakeTesting extends LinearOpMode {

    // --- Mechanisms ---
    RadahnTurretSystem turretSystem;
    RadahnHoodedOuttakeSystem hoodedOuttakeSystem;

    // --- Vision ---
    OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;

    // --- Runtime ---
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;

    // --- Camera Calibration ---
    static final double TAG_SIZE = 0.0762; // 3 inches in meters
    static final double FX = 578.272;
    static final double FY = 578.272;
    static final double CX = 402.145;
    static final double CY = 221.506;
    static final int CAMERA_WIDTH = 640;
    static final int CAMERA_HEIGHT = 480;

    // --- AprilTag IDs ---
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;

    // --- Alliance Toggle ---
    boolean isBlueAlliance = true;
    boolean lastToggleY = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Combine DS + Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- Initialize Systems ---
        PID_Controller turretPID = new PID_Controller(0.005, 0.01, 0.7, 0.0001);
        turretPID.tolerance = 0.02;

        turretSystem = new RadahnTurretSystem(
                hardwareMap, telemetry, 1.0, turretPID, CAMERA_WIDTH, Math.toRadians(60)
        );

        hoodedOuttakeSystem = new RadahnHoodedOuttakeSystem(gamepad1, telemetry, hardwareMap);

        // --- Vision Pipeline ---
        pipeline = new AprilTagDetectionPipeline(TAG_SIZE, FX, FY, CX, CY);
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "Failed to open: " + errorCode);
                telemetry.update();
            }
        });

        // --- INIT LOOP ---
        while (opModeInInit()) {
            if ((gamepad1.y != lastToggleY) && gamepad1.y) {
                isBlueAlliance = !isBlueAlliance;
            }
            lastToggleY = gamepad1.y;

            telemetry.addLine("Waiting for start...");
            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            telemetry.update();
        }

        // --- RUN LOOP ---
        while (opModeIsActive()) {
            int targetTagID = isBlueAlliance ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;
            List<AprilTagDetection> detections = pipeline.getLatestDetections();

            // --- TURRET LOGIC ---
            turretSystem.update(detections, targetTagID);

            double manualPower = 0;
            if (gamepad1.left_bumper) manualPower = -0.2;
            else if (gamepad1.right_bumper) manualPower = 0.2;
            if (manualPower != 0) turretSystem.getTurret().setPower(manualPower);

            // --- HOOD LOGIC ---
            double tagDistanceInches = 0;
            int trackedTagID = -1;
            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    if (tag.id == targetTagID) {
                        tagDistanceInches = tag.pose.z * 39.3701; // meters → inches
                        trackedTagID = tag.id;
                        break;
                    }
                }
            }

            hoodedOuttakeSystem.updateDistance(tagDistanceInches);
            hoodedOuttakeSystem.controllerInput();
            hoodedOuttakeSystem.setPositions();

            // --- TELEMETRY ---
            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            telemetry.addData("Tracked Tag ID", trackedTagID >= 0 ? trackedTagID : "None");
            telemetry.addData("Tag Distance (in)", "%.2f", tagDistanceInches);
            telemetry.addData("Loop Time", runtime.seconds() - previousTime);

            // Turret data
            telemetry.addData("Turret Encoder Ticks", turretSystem.getTurret().motors[0].getCurrPosTicks());
            telemetry.addData("Turret Angle (rad)", turretSystem.getTurret().getCurrentAngle());
            telemetry.addData("Turret Manual Power", manualPower);

            telemetry.update();

            previousTime = runtime.seconds();
        }

        // --- SHUTDOWN ---
        turretSystem.getTurret().setPower(0);
        hoodedOuttakeSystem.updateDistance(0);
        hoodedOuttakeSystem.setPositions();

        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
}
