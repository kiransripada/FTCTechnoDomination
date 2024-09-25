package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;
@TeleOp(name="SlidesPT", group="TeleOp")
public class SlidePT {
    private final DcMotor SlideMotor1;
    private final DcMotor SlideMotor2;
    private RobotParametersPT params;

    public SlidePT(RobotParametersPT params, HardwareMap hardwareMap){
        SlideMotor1 = hardwareMap.get(DcMotor.class, params.slideMotorName1);
        SlideMotor2 = hardwareMap.get(DcMotor.class, params.slideMotorName2);

    }

    public void stateUpdate(RobotParametersPT.SlideState slideState, double power) {
        switch(slideState){
            case SLIDE_IN:
                slideIn(power);
                break;

            case SLIDE_OUT:
                slideOut1(power);
                slideOut2(power);
                break;

            case STOP:
                stop1();
                stop2();
                break;
        }
    }

    public void slideIn(double power){
        SlideMotor1.setPower(power);
        SlideMotor2.setPower(power);
    }

    public void slideOut1(double power){
        SlideMotor1.setPower(-power);
    }
    public void slideOut2(double power){
        SlideMotor2.setPower(-power);
    }

    public void stop1(){
        SlideMotor1.setPower(0);
    }
    public void stop2(){
        SlideMotor1.setPower(0);
    }

}
