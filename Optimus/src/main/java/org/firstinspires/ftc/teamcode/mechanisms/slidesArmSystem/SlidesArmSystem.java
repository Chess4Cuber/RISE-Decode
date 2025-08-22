package org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlidesArmSystem {
    Gamepad gamepad1;
    OptimusSlides slides;
    OptimusArm arm;

    boolean lastToggleX = false;
    boolean lastToggleB = false;
    boolean lastToggleUp = false;
    boolean lastToggleDown = false;

    public SlidesStates slidesState;

    public SlidesArmSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        slides = new OptimusSlides(gamepad1, telemetry, hardwareMap);
        arm = new OptimusArm(hardwareMap);

        slidesState = SlidesStates.GROUND;
        this.gamepad1 = gamepad1;
    }

    public void setPositions(){
        switch (slidesState){
            case GROUND:
                slides.setExtension(0);
                arm.setPosition(0.2,0); // first arm stage
                arm.setPosition(0.2,1); //wrist
                break;
            case LOW_JUNCTION:
                slides.setExtension(0.8);
                arm.setPosition(0.7,0); // first arm stage
                arm.setPosition(0.8,1); //wrist
                break;
            case MIDDLE_JUNCTION:
                slides.setExtension(10.3);
                arm.setPosition(0.7,0); // first arm stage
                arm.setPosition(0.8,1); //wrist
                break;
            case HIGH_JUNCTION:
                slides.setExtension(18.4);
                arm.setPosition(0.7,0); // first arm stage
                arm.setPosition(0.8,1); //wrist
                break;
            case MANUAL:
                if (gamepad1.dpad_up){
                    slides.setPower(0.25);
                } else if (gamepad1.dpad_down){
                    slides.setPower(-0.25);
                } else {
                    slides.setPower(0);
                }
                break;
        }
    }

    public void controllerInput(){
        switch (slidesState) {
            case GROUND:
                if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                    slidesState = SlidesStates.HIGH_JUNCTION;
                }

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                    slidesState = SlidesStates.LOW_JUNCTION;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                    slidesState = SlidesStates.MANUAL;
                }
                break;

            case LOW_JUNCTION:
                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                    slidesState = SlidesStates.MIDDLE_JUNCTION;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                    slidesState = SlidesStates.GROUND;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                    slidesState = SlidesStates.MANUAL;
                }
                break;
            case MIDDLE_JUNCTION:
                if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                    slidesState = SlidesStates.GROUND;
                }

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up) {
                    slidesState = SlidesStates.HIGH_JUNCTION;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                    slidesState = SlidesStates.LOW_JUNCTION;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                    slidesState = SlidesStates.MANUAL;
                }
                break;
            case HIGH_JUNCTION:
                if ((gamepad1.x != lastToggleX) && gamepad1.x) {
                    slidesState = SlidesStates.GROUND;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down) {
                    slidesState = SlidesStates.MIDDLE_JUNCTION;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b) {
                    slidesState = SlidesStates.MANUAL;
                }
                break;
        }
    }
}
