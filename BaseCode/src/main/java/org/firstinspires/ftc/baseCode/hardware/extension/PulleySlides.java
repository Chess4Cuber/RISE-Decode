package org.firstinspires.ftc.baseCode.hardware.extension;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.baseCode.hardware.Motor;

public abstract class PulleySlides {

    public Motor[] motors;

    public double pulleyRadius;
    public double kGravity = 0;

    public double height;
    public double stages;

    public enum RiggingMethod {
        CONTINUOUS,
        CASCADE
    }

    public RiggingMethod method;

    PID_Controller slidesPID;

    public PulleySlides(int motorCount, String[] names, double pulleyRadius, double cpr, RiggingMethod method, int stages, double kGravity, HardwareMap hardwareMap){
        Motor[] motors = new Motor[motorCount];

        for (int i = 0; i <motors.length; i++){
            motors[i] = new Motor(names[i], cpr, hardwareMap);
            motors[i].reset();
        }

        this.pulleyRadius = pulleyRadius;

        this.method = method;
        this.stages = stages;
        this.motors = motors;
        this.kGravity = kGravity;
    }

    public void setPower(double power){
        for (Motor motor:motors) {
            motor.setPower(kGravity + power);
        }
    }

    public void setExtension(double pos){
        double slidesPower = slidesPID.PID_Power(getExtension(), pos);
        setPower(slidesPower);
    }

    public double getExtension(){
        if (method == RiggingMethod.CONTINUOUS){
            height = Math.toRadians(motors[0].getCurrPosDegrees()) * pulleyRadius;
        } else if (method == RiggingMethod.CASCADE){
            height = stages*(motors[0].getCurrPosRadians() * pulleyRadius);
        }

        return height;
    }

    public void setSlidesPID(PID_Controller slidesPID){
        this.slidesPID = slidesPID;
    }
}
