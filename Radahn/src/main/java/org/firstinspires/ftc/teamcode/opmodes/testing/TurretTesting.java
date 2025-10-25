package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.CameraVision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.mechanisms.turretSystem.RadahnTurretSystem;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp
public class TurretTesting extends LinearOpMode {

    RadahnTurretSystem turretSystem;

    OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;

    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;

    // Camera constants
    private static final double TAG_SIZE = 0.0508; // 2 inches in meters
    private static final double FX = 578.272;
    private static final double FY = 578.272;
    private static final double CX = 402.145;
    private static final double CY = 221.506;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    // Alliance toggle
    private boolean isBlueAlliance = true;
    private boolean lastToggleY = false;

    // Define the IDs of the goal tags for each alliance
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;

    @Override
    public void runOpMode() throws InterruptedException {

        // PID for turret
        PID_Controller turretPID = new PID_Controller(0.005, 0.01, 0.7, 0.0001);
        turretPID.tolerance = 0.02; // radians tolerance

        // Initialize turret system
        turretSystem = new RadahnTurretSystem(hardwareMap, telemetry, 1.0, turretPID, CAMERA_WIDTH, Math.toRadians(60));

        // Initialize camera and pipeline
        pipeline = new AprilTagDetectionPipeline(TAG_SIZE, FX, FY, CX, CY);
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "Failed to open: " + errorCode);
                telemetry.update();
            }
        });

        // Wait for start
        while (opModeInInit()) {
            if ((gamepad1.y != lastToggleY) && gamepad1.y) {
                isBlueAlliance = !isBlueAlliance;
            }
            lastToggleY = gamepad1.y;

            telemetry.addLine("Waiting for start");
            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            telemetry.update();
        }

        // Main loop
        while (opModeIsActive()) {

            // Determine target tag ID based on alliance
            int targetTagID = isBlueAlliance ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;

            // Get latest detections
            List<AprilTagDetection> detections = pipeline.getLatestDetections();

            // Update turret system
            turretSystem.update(detections, targetTagID);

            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            telemetry.addData("Loop Time", runtime.seconds() - previousTime);
            telemetry.update();
            previousTime = runtime.seconds();
        }

        // Stop turret on exit
        turretSystem.getTurret().setPower(0);
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
}
