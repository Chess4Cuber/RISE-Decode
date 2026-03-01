package org.firstinspires.ftc.teamcode.mechanisms.flywheelHoodSystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RadahnHoodedOuttakeSystem {

    RadahnHoodedOuttake hoodedOuttake;
    Gamepad gamepad1;
    Telemetry telemetry;

    boolean lastToggleB = false;
    boolean lastToggleBack = false;

    TurretHoodStates outtakeState;

    double tagDistanceInches = 0;
    double lastValidDistance = 0;

    public RadahnHoodedOuttakeSystem(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        hoodedOuttake = new RadahnHoodedOuttake(gamepad1, telemetry, hardwareMap);
        outtakeState = TurretHoodStates.RESTING;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void update() {
        controllerInput();
        setPositions();
        displayTelemetry();
    }

    public void updateDistance(double distanceInches, boolean tagVisible) {
        if (tagVisible) {
            lastValidDistance = distanceInches;
        }
        tagDistanceInches = lastValidDistance;
    }

    public void setPositions() {

        double targetFlywheelCommand = computeFlywheelPower(tagDistanceInches);
        double targetHoodPosition = computeHoodPosition(tagDistanceInches);

        switch (outtakeState) {
            case RESTING:
                hoodedOuttake.setFlywheelPower(0);
                break;

            case OUTTAKING:
                // PID + feedforward control is applied inside setFlywheelPower
                hoodedOuttake.setFlywheelPower(targetFlywheelCommand);
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;

            case INTAKING:
                // Run flywheel in reverse for intake
                hoodedOuttake.setFlywheelPower(-targetFlywheelCommand);
                hoodedOuttake.setHoodPosition(targetHoodPosition);
                break;
        }
    }

    public void controllerInput() {
        switch (outtakeState) {
            case RESTING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b)
                    outtakeState = TurretHoodStates.OUTTAKING;

                if ((gamepad1.back != lastToggleBack) && gamepad1.back)
                    outtakeState = TurretHoodStates.INTAKING;
                break;

            case OUTTAKING:
                if ((gamepad1.back != lastToggleBack) && gamepad1.back)
                    outtakeState = TurretHoodStates.INTAKING;

                if ((gamepad1.b != lastToggleB) && gamepad1.b)
                    outtakeState = TurretHoodStates.RESTING;
                break;

            case INTAKING:
                if ((gamepad1.b != lastToggleB) && gamepad1.b)
                    outtakeState = TurretHoodStates.OUTTAKING;

                if ((gamepad1.back != lastToggleBack) && gamepad1.back)
                    outtakeState = TurretHoodStates.RESTING;
                break;
        }

        lastToggleBack = gamepad1.back;
        lastToggleB = gamepad1.b;
    }


    public double computeFlywheelPower(double distanceInches) {

        double absDist = Math.abs(distanceInches);

        if(absDist >39){
            return .165;
        }

        double a = 0.00001860/1.2;
        double b = 0.00907738/1.5;
        double c = 0.00260417/1.4;

        double power = (a * absDist * absDist) + (b * absDist) + c;

        power = Math.max(0.0, Math.min(1.0, power));
        power = power * .81;
        return power;
    }

    public double computeHoodPosition(double distanceInches) {
        double absDist = Math.abs(distanceInches);
        if (absDist < 10){
            return 0.0;  // Close shots - hood down
        } else if (absDist < 39) {
            return .01; // Medium shots - hood mid
        }

        return 0.09;  // Far shots - hood position for arc
    }

    public void setMotorOuttakeState(TurretHoodStates state) {
        this.outtakeState = state;
    }


    private void displayTelemetry() {
        telemetry.addData("Outtake State", outtakeState);
        telemetry.addData("Shooter Distance", "%.1f in", tagDistanceInches);

        double targetPower = computeFlywheelPower(tagDistanceInches);
        double targetSpeed = hoodedOuttake.getTargetRPM(targetPower);
        double currentSpeed = hoodedOuttake.getRPMMotor();

        telemetry.addData("Flywheel Power", "%.2f", targetPower);
        telemetry.addData("Target RPM", "%.0f", targetSpeed);
        telemetry.addData("Current RPM", "%.0f", currentSpeed);

        // Show if we're at target speed (important for knowing when to shoot)
        telemetry.addData("At Speed", hoodedOuttake.isAtTargetSpeed() ? "YES" : "NO");

        // Show RPM error for PID tuning
        double rpmError = hoodedOuttake.getAverageRPMError();
        telemetry.addData("RPM Error", "%.0f", rpmError);

        telemetry.addData("Hood Pos", "%.2f", hoodedOuttake.getHoodPosition());
    }
}