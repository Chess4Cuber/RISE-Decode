package org.firstinspires.ftc.teamcode.mechanisms.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class ConceptScanMotif extends OpMode {

    AprilTagProcessor processor;
    VisionPortal portal;

    // Enum to represent the different AprilTag Motifs by their ID
    // 21 - GPP, 22 - PGP, 23 - PPG
    public enum Motif {
        GPP(21),
        PGP(22),
        PPG(23);

        public final int id;

        Motif(int id) {
            this.id = id;
        }

        public static Motif fromId(int id) {
            for (Motif motif : Motif.values()) {
                if (motif.id == id) {
                    return motif;
                }
            }
            return null;
        }
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Create the AprilTag processor and Vision Portal
        processor = AprilTagProcessor.easyCreateWithDefaults();
        portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                processor
        );

        telemetry.update(); // Good practice to update after init as well
    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections = processor.getDetections();

        if (!detections.isEmpty()) {
            // Get the first detected tag
            AprilTagDetection detectedTag = detections.get(0);
            Motif motif = Motif.fromId(detectedTag.id);

            if (motif != null) {
                telemetry.addData("Motif", motif.name());
                // Display the telemetry for the detected tag
                tagToTelemetry(detectedTag);
            }
        } else {
            telemetry.addData("Motif", "None");
        }

        // Update the telemetry on the Driver Station
        telemetry.update();
    }

    /**
     * Adds telemetry lines for an AprilTag detection.
     * @param detection The AprilTag detection to display.
     */
    private void tagToTelemetry(AprilTagDetection detection) {
        // Check if the pose data is available for the detection

            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            //telemetry.addLine(String.format("Translation X: %.2f inches", detection.ftcPose.x));
            telemetry.addLine(String.format("Translation Y: %.2f inches", detection.ftcPose.y));
            telemetry.addLine(String.format("Translation Z: %.2f inches", detection.ftcPose.z));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", detection.ftcPose.yaw));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", detection.ftcPose.pitch));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", detection.ftcPose.roll));

    }
}
