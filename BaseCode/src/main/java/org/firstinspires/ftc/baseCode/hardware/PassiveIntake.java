package org.firstinspires.ftc.baseCode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class PassiveIntake {
    Gamepad gamepad1;
    Telemetry telemetry;
    public Motor[] motors;

    public PassiveIntake(int motorCount, String[] name, double cpr, Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        Motor[] motors = new Motor[motorCount];
        for (int i = 0; i < motors.length; i++){
            motors[i] = new Motor(name[i], cpr, hardwareMap);
            if(i%2 != 0){
                motors[i].setDirectionReverse();
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else{
                motors[i].setDirectionForward();
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            motors[i].reset();
        }

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        this.motors = motors;
    }

    public PassiveIntake(String name, double cpr, Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        motors = new Motor[]{new Motor(name, cpr, hardwareMap)};

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void toggleIntake(){
        for (Motor motor : motors) {
            motor.setPower(Math.pow(3, gamepad1.right_trigger - gamepad1.left_trigger));
            motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }

    public void setPower(double power){
        for (Motor motor : motors) {
            motor.setPower(power);
        }
    }

    public void setVelocity(double velocity){
        for (Motor motor : motors) {
            motor.setPower(velocity);
        }
    }

    //TODO: make intake position method using encoders
}
