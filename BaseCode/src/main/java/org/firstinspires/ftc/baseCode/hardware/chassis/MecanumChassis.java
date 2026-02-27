package org.firstinspires.ftc.baseCode.hardware.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.baseCode.control.motionProfiling.TrapezoidalMotionProfile;
import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.Motor;
import org.firstinspires.ftc.baseCode.math.Vector3D;
import org.firstinspires.ftc.baseCode.sensors.odometry.Odometry;
import org.firstinspires.ftc.baseCode.sensors.odometry.OdometryType;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public abstract class MecanumChassis {

    Gamepad gamepad1;
    Telemetry telemetry;

    public Motor frontLeft;
    public Motor frontRight;
    public Motor backRight;
    public Motor backLeft;


    double fLeft;
    double fRight;
    double bLeft;
    double bRight;

    double max;

    double VX_WEIGHT = 1;
    double VY_WEIGHT = 1;
    double OMEGA_WEIGHT = 1;

    public Odometry odo;

    PID_Controller TranslationalPID_X;
    PID_Controller TranslationalPID_Y;
    PID_Controller HeadingPID;

    public TrapezoidalMotionProfile TranslationalProfile_X;
    public TrapezoidalMotionProfile TranslationalProfile_Y;
    public TrapezoidalMotionProfile TranslationalProfile_Heading;

    Vector3D controllerInput;

    public MecanumChassis(String[] motorNames, Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        frontLeft = new Motor(motorNames[0], hardwareMap);
        frontRight = new Motor(motorNames[1], hardwareMap);
        backRight = new Motor(motorNames[2], hardwareMap);
        backLeft = new Motor(motorNames[3], hardwareMap);

        frontRight.setDirectionReverse();
        backRight.setDirectionReverse();
        frontLeft.setDirectionForward();
        backLeft.setDirectionForward();

        List<Motor> motors = Arrays.asList(frontLeft, frontRight, backRight, backLeft);

        for (Motor motor : motors) {
            motor.setBreakMode();
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.reset();
        }

        controllerInput = new Vector3D(0,0,0);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void setDriveVectorsRobotCentric(Vector3D input)  {
        fLeft = -VX_WEIGHT * input.A - VY_WEIGHT * input.B - OMEGA_WEIGHT * input.C;
        fRight = -VX_WEIGHT * input.A + VY_WEIGHT * input.B + OMEGA_WEIGHT * input.C;
        bRight = VX_WEIGHT * input.A - VY_WEIGHT * input.B + OMEGA_WEIGHT * input.C;
        bLeft = VX_WEIGHT * input.A + VY_WEIGHT * input.B - OMEGA_WEIGHT * input.C;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        setPower(fLeft, fRight, bRight, bLeft);
    }

    public void setDriveVectorsFieldCentric(Vector3D input){
        double x_rotated = input.A * Math.cos(odo.getHeading()) - input.B * Math.sin(odo.getHeading());
        double y_rotated = input.A * Math.sin(odo.getHeading()) + input.B * Math.cos(odo.getHeading());

        fLeft  = -VX_WEIGHT * x_rotated - VY_WEIGHT * y_rotated - OMEGA_WEIGHT * input.C;
        fRight = -VX_WEIGHT * x_rotated + VY_WEIGHT * y_rotated + OMEGA_WEIGHT * input.C;
        bRight =  VX_WEIGHT * x_rotated - VY_WEIGHT * y_rotated + OMEGA_WEIGHT * input.C;
        bLeft  =  VX_WEIGHT * x_rotated + VY_WEIGHT * y_rotated - OMEGA_WEIGHT * input.C;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        setPower(fLeft, fRight, bRight, bLeft);
    }

    public void robotCentricDrive(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsRobotCentric(controllerInput);
    }

    public void fieldCentricDrive(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsFieldCentric(controllerInput);
    }

    public void goToPosePID(Vector3D input){
        // Translation PIDs
        double PID_Drive = TranslationalPID_X.PID_Power(odo.getX(), input.A);
        double PID_Strafe = TranslationalPID_Y.PID_Power(odo.getY(), input.B);

        // Heading PID
        double currentHeading = getPose()[2];   // in degrees
        double targetHeading = input.C;         // in degrees

        // Wrap the heading error to [-180, 180] to take shortest rotation path
        double headingError = targetHeading - currentHeading;
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;

        // Feed PID: current = 0, target = headingError
        double PID_Turn = HeadingPID.PID_Power(0, headingError);

        Vector3D PID_Vector = new Vector3D(PID_Drive, PID_Strafe, PID_Turn);

        setDriveVectorsFieldCentric(PID_Vector);
    }



    public void goToPoseTrapezoidalProfile(Vector3D input){
        double Profile_Drive = TranslationalProfile_X.getPosProfilePower(odo.getX(), input.A);
        double Profile_Strafe = TranslationalProfile_Y.getPosProfilePower(odo.getY(), input.B);
        double Profile_Turn = TranslationalProfile_Heading.getPosProfilePower(angleWrap(odo.getHeading()), input.C);

        Vector3D Profile_Vector = new Vector3D(Profile_Drive, Profile_Strafe, Profile_Turn);

        setDriveVectorsFieldCentric(Profile_Vector);
    }

    public double angleWrap(double radians) {
        if (radians > Math.PI) {
            radians -= 2*Math.PI;
        }
        if (radians < -Math.PI) {
            radians += 2*Math.PI;
        }
        return radians;
    }

    public void setPID_Controller(PID_Controller TranslationalPID_X, PID_Controller TranslationalPID_Y, PID_Controller HeadingPID){
        this.TranslationalPID_X = TranslationalPID_X;
        this.TranslationalPID_Y = TranslationalPID_Y;
        this.HeadingPID = HeadingPID;
    }

    public void setTrapezoidalMotionProfile(TrapezoidalMotionProfile TranslationalProfile_X, TrapezoidalMotionProfile TranslationalProfile_Y, TrapezoidalMotionProfile TranslationalProfile_Heading){
        this.TranslationalProfile_X = TranslationalProfile_X;
        this.TranslationalProfile_Y = TranslationalProfile_Y;
        this.TranslationalProfile_Heading = TranslationalProfile_Heading;
    }

    public void setOdometry(String[] odoNames, OdometryType odoType, double[] odoConstants, HardwareMap hardwareMap, double xOff, double yOff, double headOff){
        odo = new Odometry(odoNames, odoType, odoConstants, hardwareMap, xOff, yOff, headOff);
    }

    public double[] getPose(){
        return odo.getPose();
    }

    public double[] getEncoderReadings(){
        return odo.getEncoderReadings();
    }
    public Vector3D getPoseVector(){
        return odo.getPoseVector();
    }

    public void updatePose(){
        odo.updatePose();
    }
    public void setPower(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backRight.setPower(power);
        this.backLeft.setPower(power);
    }

    public void setPower(double fLeft, double fRight, double bRight, double bLeft){
        this.frontLeft.setPower(fLeft);
        this.frontRight.setPower(fRight);
        this.backRight.setPower(bRight);
        this.backLeft.setPower(bLeft);
    }

    public void reset(){
        this.frontLeft.reset();
        this.frontRight.reset();
        this.backRight.reset();
        this.backLeft.reset();
    }

    public void setBreakMode(){
        this.frontLeft.setBreakMode();
        this.frontRight.setBreakMode();
        this.backRight.setBreakMode();
        this.backLeft.setBreakMode();
    }

    public void setFloatMode(){
        this.frontLeft.setFloatMode();
        this.frontRight.setFloatMode();
        this.backRight.setFloatMode();
        this.backLeft.setFloatMode();
    }

    public void setWeights(double[] chassisWeights){
        VX_WEIGHT = chassisWeights[0];
        VY_WEIGHT = chassisWeights[1];
        OMEGA_WEIGHT = chassisWeights[2];
    }
}