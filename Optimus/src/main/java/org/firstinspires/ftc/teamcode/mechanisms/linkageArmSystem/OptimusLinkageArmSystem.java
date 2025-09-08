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
    public LinkageStates linkageState;
    boolean lastToggleA = false;
    public OptimusLinkageArmSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        linkage = new OptimusLinkage(gamepad1, telemetry, hardwareMap);

        this.gamepad1 = gamepad1;
        linkageState = LinkageStates.RETRACTED;

    }

    public void setPositions(){
        switch (linkageState) {
            case RETRACTED:
                linkage.setPosition(.47, 0);
                break;
            case EXTENDED:
                linkage.setPosition(.7, 0);
                break;
        }
    }

    public void controllerInput(){
        switch (linkageState) {
            case RETRACTED:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    linkageState = LinkageStates.EXTENDED;
                }

                break;
            case EXTENDED:
                if ((gamepad1.a != lastToggleA) && gamepad1.a) {
                    linkageState = LinkageStates.RETRACTED;
                }
                break;
        }
        lastToggleA = gamepad1.a;
    }

}
