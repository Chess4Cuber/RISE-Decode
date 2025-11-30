package org.firstinspires.ftc.teamcode.opmodes.auton;

import static org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.MotorIntakeStates.INTAKING;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.baseCode.math.Vector3D;
import org.firstinspires.ftc.teamcode.mechanisms.RadahnChassis;
import org.firstinspires.ftc.teamcode.mechanisms.motorIntakeSystem.RadahnMotorIntakeSystem;

@Autonomous
public class Nine_Ball_BLUE extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    int cycleCounter = 1;

    public enum AutoStep{
        AWAY_FROM_GOAL,
        SHOOTPRE,
        FIRST_LINE,
        SHOOT_FIRST,
        SECOND_LINE,
        SHOOT_SECOND,
        THIRD_LINE,
        SHOOT_THIRD,

    }

    RadahnChassis chassis;
    RadahnMotorIntakeSystem intake;
    AutoStep parkingStep;

    Vector3D poseVector = new Vector3D(0,0,0);

    Vector3D targetPose = new Vector3D(0, 0, 0);
    double tolerance = 1.5;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new RadahnChassis(gamepad1, telemetry, hardwareMap);
        intake = new RadahnMotorIntakeSystem(gamepad1, telemetry, hardwareMap);

        parkingStep = AutoStep.AWAY_FROM_GOAL;

        while (opModeInInit()){

            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.updatePose();

            chassis.goToPosePID(targetPose);
            poseVector.set(chassis.odo.getX(), chassis.odo.getY(), chassis.getPose()[2]);

            intake.setPositions();

            autonLeftRed();

            Telemetry();

            telemetry.update();
        }
    }

    public void autonLeftRed(){
        switch (parkingStep){
            case AWAY_FROM_GOAL:
                targetPose.set(-35, -70, 63);

                if (targetPose.findDistance(poseVector) < tolerance ){
                    parkingStep = AutoStep.SHOOTPRE;
                    runtime.reset();
                }
                break;

            case SHOOTPRE:
                //targetPose.set(0, -63, 60);

                //TODO: ADD SHOOTING SUBSYSTEMS

                if (runtime.seconds() > 4 ){
                    parkingStep = AutoStep.FIRST_LINE;
                    runtime.reset();
                }

                intake.setMotorIntakeState(INTAKING);

                parkingStep = AutoStep.FIRST_LINE;

                break;

            case FIRST_LINE:
                targetPose.set(-60, -20, 63);
                break;
        }
    }

    public void Telemetry(){
        //TODO: show the tolerance stuff and PID values and states
        telemetry.addData("Auton State", parkingStep);

        telemetry.addData("Pose Estimate X:", chassis.getPose()[0]);
        telemetry.addData("Pose Estimate Y:", chassis.getPose()[1]);
        telemetry.addData("Pose Estimate Angle:", chassis.getPose()[2]);

        telemetry.addData("HeadingPID error", chassis.HeadingPID.error);
        telemetry.addData("TranslationalX error", chassis.TranslationalPID_X.error);
        telemetry.addData("TranslationalY error", chassis.TranslationalPID_Y.error);
        telemetry.addData("Distance to Pose", targetPose.findDistance(chassis.getPoseVector()));

        telemetry.addData("X PID Proportion", chassis.TranslationalPID_X.P);
        telemetry.addData("Y PID Proportion", chassis.TranslationalPID_Y.P);
        telemetry.addData("Heading PID Proportion", chassis.HeadingPID.P);

        telemetry.addData("TranslationalX PID Derivative", chassis.TranslationalPID_X.D);
        telemetry.addData("TranslationalY PID Derivative", chassis.TranslationalPID_Y.D);

        telemetry.addData("X PID Integral", chassis.TranslationalPID_X.I);
        telemetry.addData("Y PID Integral", chassis.TranslationalPID_Y.I);
        telemetry.addData("Heading PID Integral", chassis.HeadingPID.I);


        telemetry.addData("runtime", runtime.seconds());

    }
}
