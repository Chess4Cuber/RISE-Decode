package org.firstinspires.ftc.baseCode.CameraVision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();
    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();

    Mat cameraMatrix;

    double fx, fy, cx, cy;
    double tagsize;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public AprilTagDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy) {
        this.tagsize = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        constructMatrix();

        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(
                AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public void finalize() {
        if(nativeApriltagPtr != 0) {
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync) {
            if(needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        // Draw overlays on the camera feed
        for (AprilTagDetection detection : detections) {
            // Draw bounding box
            for (int i = 0; i < 4; i++) {
                Point pt1 = detection.corners[i];
                Point pt2 = detection.corners[(i + 1) % 4];
                Imgproc.line(input, pt1, pt2, new Scalar(0, 255, 0), 3);
            }
            // Draw center
            Imgproc.circle(input, detection.center, 5, new Scalar(0, 0, 255), -1);

            // Draw 6DOF axes and cube if needed
            Pose pose = aprilTagPoseToOpenCvPose(detection.pose);
            drawAxisMarker(input, tagsize/2.0, 4, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagsize, tagsize, tagsize, 3, pose.rvec, pose.tvec, cameraMatrix);
        }

        // Draw vertical line at frame center
        Imgproc.line(input, new Point(cx, 0), new Point(cx, input.rows()), new Scalar(255, 0, 0), 2);

        return input;
    }

    public void setDecimation(float decimation) {
        synchronized (decimationSync) {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }

    void constructMatrix() {
        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);
        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1, fy);
        cameraMatrix.put(1,2, cy);

        cameraMatrix.put(2,0,0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );
        MatOfPoint2f projected = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), projected);
        Point[] pts = projected.toArray();
        Imgproc.line(buf, pts[0], pts[1], new Scalar(255,0,0), thickness);
        Imgproc.line(buf, pts[0], pts[2], new Scalar(0,255,0), thickness);
        Imgproc.line(buf, pts[0], pts[3], new Scalar(0,0,255), thickness);
        Imgproc.circle(buf, pts[0], thickness, new Scalar(255,255,255), -1);
    }

    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2,-tagHeight/2,-length),
                new Point3(-tagWidth/2,-tagHeight/2,-length));
        MatOfPoint2f projected = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), projected);
        Point[] pts = projected.toArray();
        for(int i=0;i<4;i++){
            Imgproc.line(buf, pts[i], pts[i+4], new Scalar(0,0,255), thickness);
        }
        Imgproc.line(buf, pts[4], pts[5], new Scalar(0,255,0), thickness);
        Imgproc.line(buf, pts[5], pts[6], new Scalar(0,255,0), thickness);
        Imgproc.line(buf, pts[6], pts[7], new Scalar(0,255,0), thickness);
        Imgproc.line(buf, pts[4], pts[7], new Scalar(0,255,0), thickness);
    }

    Pose aprilTagPoseToOpenCvPose(AprilTagPose aprilTagPose) {
        Pose pose = new Pose();
        pose.tvec.put(0,0, aprilTagPose.x);
        pose.tvec.put(1,0, aprilTagPose.y);
        pose.tvec.put(2,0, aprilTagPose.z);
        Mat R = new Mat(3,3,CvType.CV_32F);
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                R.put(i,j, aprilTagPose.R.get(i,j));
        Calib3d.Rodrigues(R, pose.rvec);
        return pose;
    }

    class Pose {
        Mat rvec;
        Mat tvec;
        Pose() {
            rvec = new Mat(3,1,CvType.CV_32F);
            tvec = new Mat(3,1,CvType.CV_32F);
        }
    }
}
