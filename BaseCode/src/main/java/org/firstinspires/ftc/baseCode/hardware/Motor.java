package org.firstinspires.ftc.baseCode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Motor {
    public DcMotor dcMotorEx;

    //Declare all the constants in the Motor class
    public double TICKS_PER_REV = 0;
    public double WHEEL_DIAMETER = 0;
    public double TICKS_PER_INCH = 0;
    public double TICKS_PER_RADIAN = 0;
    public double TICKS_PER_DEGREE = 0;
    public double NANOSECONDS_PER_MIN = 6e+10;
    public double GearRatio;

       /* Constructor for drive train motors
       Parameter name : Pass in name of the motor on the RC phone config
       Parameter hwmap : Pass in the hardwareMap from OpMode to initialize the motor */

    /* Constructor for drive train motors
       Parameter name : Pass in name of the motor on the RC phone config
       Parameter hwmap : Pass in the hardwareMap from OpMode to initialize the motor */

    PID_Controller motorVeloPID;

    public Motor(String name, HardwareMap hardwareMap){
        dcMotorEx = hardwareMap.get(DcMotor.class, name);

        motorVeloPID = new PID_Controller(0);
    }

    public Motor(String name , double cpr, HardwareMap hwmap){
        dcMotorEx = hwmap.get(DcMotorEx.class, name);
        TICKS_PER_REV = cpr;
        TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
        TICKS_PER_DEGREE = (TICKS_PER_REV / 360);
        TICKS_PER_RADIAN = (TICKS_PER_REV / (2*Math.PI));
        
        motorVeloPID = new PID_Controller(0);
    }

    public Motor(String name , double cpr, double kp, double kd, double ki, HardwareMap hwmap){
        dcMotorEx = hwmap.get(DcMotorEx.class, name);
        this.TICKS_PER_REV = cpr;
        TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
        TICKS_PER_DEGREE = (TICKS_PER_REV / 360);
        TICKS_PER_RADIAN = (TICKS_PER_REV / (2*Math.PI));

        motorVeloPID = new PID_Controller(kp, kd, ki);
    }

    public void reset(){
        dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        dcMotorEx.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setBreakMode(){
        dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setFloatMode(){
        dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double getCurrPosDegrees(){
        return (dcMotorEx.getCurrentPosition()/TICKS_PER_DEGREE);
    }
    public double getCurrPosRadians(){
        return (dcMotorEx.getCurrentPosition()/TICKS_PER_RADIAN);
    }

    public double getCurrPosTicks(){
        return dcMotorEx.getCurrentPosition();
    }

    public double getCurrPosInches(){
        return dcMotorEx.getCurrentPosition()/TICKS_PER_INCH;
    }


    public MotorConfigurationType getMotorType(){
        return dcMotorEx.getMotorType();
    }

    public void setMotorType(MotorConfigurationType motorConfigurationType){
        dcMotorEx.setMotorType(motorConfigurationType);
    }

    public void setDirectionForward(){
        dcMotorEx.setDirection(DcMotor.Direction.FORWARD);
    }
    public void setDirectionReverse(){
        dcMotorEx.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPower(double power){
        dcMotorEx.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode) {
        dcMotorEx.setMode(runMode);
    }
}
