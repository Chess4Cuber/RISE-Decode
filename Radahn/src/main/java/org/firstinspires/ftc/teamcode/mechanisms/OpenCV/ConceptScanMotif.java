package org.firstinspires.ftc.teamcode.mechanisms.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.baseCode.CameraVision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.baseCode.CameraVision.AprilTagTracker;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;


@Autonomous
public class ConceptScanMotif extends OpMode {

    // --- Vision configuration ---
    private static final double TAG_SIZE = 0.0508; // 2 inches (in meters)
    private static final double FX = 578.272;
    private static final double FY = 578.272;

    private static final double CX = 402.145;
    private static final double CY = 221.506;

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    // --- Motor configuration ---
    private static final double PAN_MOTOR_CPR = 387.5;  // encoder counts per revolution

    OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;
    AprilTagTracker tracker;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        pipeline = new AprilTagDetectionPipeline(TAG_SIZE, FX, FY, CX, CY);

        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(pipeline);

        // Open camera and start streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "Failed to open: " + errorCode);
            }
        });

        // Initialize the AprilTag tracker
        tracker = new AprilTagTracker("panMotor", PAN_MOTOR_CPR, hardwareMap, CAMERA_WIDTH);
        tracker.setDirectionReversed(); // flip if needed
        tracker.setStopRangePixels(30); // stop when tag is centered ±30px
        tracker.setMaxPower(0.18);      // limit max speed for smoother movement

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections = pipeline.getLatestDetections();

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);

            // Move motor to center tag
            double power = tracker.trackTag(detections);

            double xIn = tag.pose.x * 39.3701;
            double yIn = tag.pose.y * 39.3701;
            double zIn = tag.pose.z * 39.3701;

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("X (in)", "%.2f", xIn);
            telemetry.addData("Y (in)", "%.2f", yIn);
            telemetry.addData("Z (in)", "%.2f", zIn);
            telemetry.addData("Error (px)", "%.1f", tracker.getFrameCenterX() - tag.center.x);
            telemetry.addData("Motor Power", "%.3f", power);
        } else {
            tracker.stopTracking();
            telemetry.addData("Tag", "None detected");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        tracker.stopTracking();
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
}
