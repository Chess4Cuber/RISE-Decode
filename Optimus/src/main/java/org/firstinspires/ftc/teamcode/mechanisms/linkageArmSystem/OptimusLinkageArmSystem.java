package org.firstinspires.ftc.teamcode.mechanisms.linkageArmSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.intakeSystem.IntakeStates;
import org.firstinspires.ftc.teamcode.mechanisms.intakeSystem.OptimusIntake;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.OptimusArm;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.OptimusSlides;
import org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem.SlidesStates;

public class OptimusLinkageArmSystem {

    Gamepad gamepad1;
    OptimusLinkage linkage;
    OptimusLinkage intakeRotator;
    OptimusIntake intake;
    public IntakeStates intakeState;

    public LinkageStates linkageState;
    boolean lastToggleA = false;
    public OptimusLinkageArmSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        linkage = new OptimusLinkage(gamepad1, telemetry, hardwareMap);
        intakeRotator = new OptimusLinkage(gamepad1, telemetry, hardwareMap);
        intake = new OptimusIntake(hardwareMap);
        intakeState = IntakeStates.RESTING;


        this.gamepad1 = gamepad1;
        linkageState = LinkageStates.RETRACTED;

    }

    public void setPositions(){
        switch (linkageState) {
            case RETRACTED:
                linkage.setPosition(.47, 0);
                intakeRotator.setPosition(.47, 1);
                intake.setPower(0);
                break;
            case EXTENDED:
                linkage.setPosition(.7, 0);
                intakeRotator.setPosition(.87, 1);
                intake.setPower(1);
                break;
        }
    }

    public void controllerInput(){
        switch (linkageState) {
            case RETRACTED:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    linkageState = LinkageStates.EXTENDED;
                    intakeState = IntakeStates.INTAKING;
                }

                break;
            case EXTENDED:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    linkageState = LinkageStates.RETRACTED;
                    intakeState = IntakeStates.RESTING;
                }
                break;
        }
        lastToggleA = gamepad1.a;
    }

}
