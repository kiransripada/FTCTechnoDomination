package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

public class SlidePT {
    private final DcMotor SlideMotor;
    private RobotParametersPT params;

    public SlidePT(RobotParametersPT params, HardwareMap hardwareMap){
        SlideMotor = hardwareMap.get(DcMotor.class, params.slideMotorName);
    }

    public void stateUpdate(RobotParametersPT.SlideState slideState, double power) {
        switch(slideState){
            case SLIDE_IN:
                slideIn(power);
                break;

            case SLIDE_OUT:
                slideOut(power);
                break;

            case STOP:
                stop();
                break;
        }
    }

    public void slideIn(double power){
        SlideMotor.setPower(power);
    }

    public void slideOut(double power){
        SlideMotor.setPower(-power);
    }

    public void stop(){
        SlideMotor.setPower(0);
    }

}
