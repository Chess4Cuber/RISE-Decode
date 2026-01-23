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

    // PID
    PID_Controller turretPID = new PID_Controller(0.02, 0.001);

    // Soft limits
    double leftLimit = -160;
    double rightLimit = 160;

    // Target angle storage
    double targetAngle = 0;

    // Limelight inputs
    double tx = 0;
    boolean targetVisible = false;

    // Manual nudge size (degrees)
    double manualStep = 5;

    boolean lastLB = false;
    boolean lastRB = false;

    public RadahnTurretSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        turret = new RadahnTurret(gamepad1, telemetry, hardwareMap);

        turretPID.tolerance = 0.5;

        // Start in auto aim mode
        turretState = TurretStates.AUTO_AIM;
    }

    // Receive Limelight data
    public void updateLimelight(double tx, boolean targetVisible){
        this.tx = tx;
        this.targetVisible = targetVisible;
    }

    public void controllerInput(){

        // Both bumpers -> auto aim
        if(gamepad1.left_bumper && gamepad1.right_bumper){
            turretState = TurretStates.AUTO_AIM;
        }

        // Left bumper nudge left
        else if((gamepad1.left_bumper != lastLB) && gamepad1.left_bumper){
            turretState = TurretStates.MANUAL;
            targetAngle -= manualStep;
        }

        // Right bumper nudge right
        else if((gamepad1.right_bumper != lastRB) && gamepad1.right_bumper){
            turretState = TurretStates.MANUAL;
            targetAngle += manualStep;
        }

        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
    }

    public void setPositions(){

        double currentAngle = turret.getAngleDegrees();

        switch(turretState){

            case RESTING:
                turret.setPower(0);
                break;

            case MANUAL:
                // PID to manual targetAngle
                double manualPower = turretPID.PID_Power(currentAngle, targetAngle);
                manualPower = clamp(manualPower, -0.5, 0.5);
                applySoftLimits(manualPower, currentAngle);
                break;

            case AUTO_AIM:

                if(targetVisible){
                    targetAngle = currentAngle + tx;
                }

                double autoPower = turretPID.PID_Power(currentAngle, targetAngle);
                autoPower = clamp(autoPower, -0.5, 0.5);
                applySoftLimits(autoPower, currentAngle);
                break;
        }

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
