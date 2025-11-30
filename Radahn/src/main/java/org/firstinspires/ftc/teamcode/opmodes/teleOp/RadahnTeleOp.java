package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.baseCode.CameraVision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnPusher;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem.RadahnSpindexerSystem;
import org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem.RadahnHoodedOuttakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.simpleMotorOuttakeSystem.RadahnMotorOuttakeSystem;
import org.firstinspires.ftc.teamcode.mechanisms.turretSystem.RadahnTurretSystem;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@TeleOp
public class RadahnTeleOp extends LinearOpMode {
    RadahnHoodedOuttakeSystem hoodedOuttakeSystem;
    RadahnChassis chassis;
    RadahnMotorIntakeSystem intake;
    RadahnPusher pusher;
    OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;


    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;

    private static final double TAG_SIZE = 0.17; // 2 inches in meters
    private static final double FX = 578.272;
    private static final double FY = 578.272;
    private static final double CX = 402.145;
    private static final double CY = 221.506;

    private static final int CAMERA_WIDTH = 176;
    private static final int CAMERA_HEIGHT = 144;

    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;

    private boolean isBlueAlliance = true;
    private boolean lastToggleY = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hoodedOuttakeSystem = new RadahnHoodedOuttakeSystem(gamepad1, telemetry, hardwareMap);
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        intake = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);
        pusher = new RadahnPusher(gamepad1, hardwareMap);

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

        while (opModeInInit()) {
            pusher.openClaw();

            if ((gamepad1.y != lastToggleY) && gamepad1.y) {
                isBlueAlliance = !isBlueAlliance;
            }
            lastToggleY = gamepad1.y;

            telemetry.addLine("Waiting For Start");
            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            telemetry.update();
        }

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = pipeline.getLatestDetections();
            double tagDistanceInches = 0;
            int trackedTagID = -1;

            if (detections != null && !detections.isEmpty()) {
                int targetTagID = isBlueAlliance ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;

                for (AprilTagDetection tag : detections) {
                    if (tag.id == targetTagID) {
                        tagDistanceInches = tag.pose.z * 39.3701; // meters -> inches
                        trackedTagID = tag.id;
                        break;
                    }
                }
            }

            hoodedOuttakeSystem.updateDistance(tagDistanceInches);
            hoodedOuttakeSystem.controllerInput();
            hoodedOuttakeSystem.setPositions();

            chassis.robotCentricDrive();
            chassis.updatePose();

            intake.controllerInput();
            intake.setPositions();

            pusher.toggleClaw();

            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            telemetry.addData("Tracked Tag ID", trackedTagID >= 0 ? trackedTagID : "None");
            telemetry.addData("Tag Distance (inches)", "%.2f", tagDistanceInches);
            telemetry.addData("Loop Time", runtime.seconds() - previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }

        hoodedOuttakeSystem.updateDistance(0);
        hoodedOuttakeSystem.setPositions();

        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
}
