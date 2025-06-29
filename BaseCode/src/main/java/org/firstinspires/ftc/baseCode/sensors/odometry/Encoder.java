package org.firstinspires.ftc.baseCode.sensors.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.baseCode.control.PID_Controller;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Encoder {
    public DcMotorEx dcMotorEx;

    //Declare all the constants in the Motor class
    public double TICKS_PER_REV;
    public double WHEEL_DIAMETER;
    public double TICKS_PER_INCH;
    public double TICKS_PER_RADIAN;
    public double TICKS_PER_DEGREE;

    public Encoder(String name , double cpr, double wheelDia, HardwareMap hwmap){
        dcMotorEx = hwmap.get(DcMotorEx.class, name);
        TICKS_PER_REV = cpr;
        WHEEL_DIAMETER = wheelDia;
        TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
        TICKS_PER_DEGREE = (TICKS_PER_REV / 360);
        TICKS_PER_RADIAN = (TICKS_PER_REV / (2*Math.PI));
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
        return getCurrPosRadians()*WHEEL_DIAMETER;
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
    public void setMode(DcMotor.RunMode runMode) {
        dcMotorEx.setMode(runMode);
    }
}
