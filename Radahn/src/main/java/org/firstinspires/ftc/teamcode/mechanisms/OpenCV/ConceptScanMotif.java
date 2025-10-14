package org.firstinspires.ftc.teamcode.mechanisms.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.baseCode.CameraVision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.baseCode.CameraVision.AprilTagTracker;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@Autonomous
public class ConceptScanMotif extends OpMode {

    private AprilTagDetectionPipeline pipeline;
    private AprilTagTracker tracker;
    private OpenCvCamera camera;

    // Camera lens intrinsics for your webcam
    private static final double FX = 578.272;
    private static final double FY = 578.272;
    private static final double CX = 402.145;
    private static final double CY = 221.506;

    // AprilTag size in meters
    private static final double TAG_SIZE = 0.052;

    // Camera resolution
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Create pipeline
        pipeline = new AprilTagDetectionPipeline(TAG_SIZE, FX, FY, CX, CY);


        // Initialize motor tracker
        tracker = new AprilTagTracker("panMotor", 387.5, hardwareMap, CAMERA_WIDTH);
        tracker.setDirectionReversed();       // adjust if motor moves backward
        tracker.setStopRangePixels(40);       // stop motor when tag is within ±40 pixels

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        List<AprilTagDetection> detections = pipeline.getLatestDetections();

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);

            // Track motor to center tag
            double motorPower = tracker.trackTag(detections);

            // Convert pose from meters to inches
            double xIn = tag.pose.x * 39.3701;
            double yIn = tag.pose.y * 39.3701;
            double zIn = tag.pose.z * 39.3701;

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("X (in)", "%.2f", xIn);
            telemetry.addData("Y (in)", "%.2f", yIn);
            telemetry.addData("Z (in)", "%.2f", zIn);
            telemetry.addData("Motor Power", "%.3f", motorPower);
            telemetry.addData("PID Error", "%.2f", tracker.getFrameCenterX() - tag.center.x);
        } else {
            tracker.stopTracking();
            telemetry.addData("Motif", "None");
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


    public void idk(){
        //        // Setup camera and display preview on REV Hub
//        int cameraMonitorViewId = hardwareMap.appContext.getResources()
//                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance()
//                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        camera.setPipeline(pipeline);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera", "Failed to open: " + errorCode);
//                telemetry.update();
//            }
//        });
    }

}
