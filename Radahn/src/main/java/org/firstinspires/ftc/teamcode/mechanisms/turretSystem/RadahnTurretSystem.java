package org.firstinspires.ftc.teamcode.mechanisms.turretSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnTurretSystem {

    Gamepad gamepad1;
    Telemetry telemetry;
    RadahnTurret turret;

    public TurretStates turretState;

    PID_Controller turretPID = new PID_Controller(0.02, 0.001);

    double leftLimit = -160;
    double rightLimit = 160;

    double targetAngle = 0;

    double tx = 0;
    boolean targetVisible = false;

    double manualStep = 5;

    boolean lastLB = false;
    boolean lastRB = false;

    public RadahnTurretSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        turret = new RadahnTurret(gamepad1, telemetry, hardwareMap);

        turretPID.tolerance = 0.5;

        turretState = TurretStates.AUTO_AIM;
    }

    // Receive Limelight data
    public void updateLimelight(double tx, boolean targetVisible){
        this.tx = tx;
        this.targetVisible = targetVisible;
    }

    public void update(){
        controllerInput();
        setPositions();
    }

    public void controllerInput(){

        switch(turretState){

            case AUTO_AIM:
            case MANUAL:
            case RESTING:

                // Both bumpers → return to auto
                if(gamepad1.left_bumper && gamepad1.right_bumper){
                    turretState = TurretStates.AUTO_AIM;
                    break;
                }

                // Left bumper → manual nudge left
                if((gamepad1.left_bumper != lastLB) && gamepad1.left_bumper){
                    turretState = TurretStates.MANUAL;
                    targetAngle -= manualStep;
                    break;
                }

                // Right bumper → manual nudge right
                if((gamepad1.right_bumper != lastRB) && gamepad1.right_bumper){
                    turretState = TurretStates.MANUAL;
                    targetAngle += manualStep;
                    break;
                }

                break;
        }

        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
    }

    public void setPositions(){

        double currentAngle = turret.getAngleDegrees();
        double power = 0;

        switch(turretState){

            case RESTING:
                power = 0;
                break;

            case MANUAL:
                power = turretPID.PID_Power(currentAngle, targetAngle);
                break;

            case AUTO_AIM:

                if(targetVisible){
                    targetAngle = currentAngle + tx;
                }

                power = turretPID.PID_Power(currentAngle, targetAngle);
                break;
        }

        power = clamp(power, -0.5, 0.5);
        applySoftLimits(power, currentAngle);
        displayTelemetry(currentAngle);
    }

    private void applySoftLimits(double power, double angle){
        if(angle <= leftLimit && power < 0) power = 0;
        if(angle >= rightLimit && power > 0) power = 0;
        turret.setPower(power);
    }

    private double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }

    private void displayTelemetry(double angle){
        telemetry.addData("Turret State", turretState);
        telemetry.addData("Turret Angle", "%.1f", angle);
        telemetry.addData("Turret Target", "%.1f", targetAngle);
        telemetry.addData("tx", "%.2f", tx);
        telemetry.addData("Target Visible", targetVisible);
    }
}
