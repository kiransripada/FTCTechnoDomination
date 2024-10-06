package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;
public class SlidePT {
    private final DcMotor SlideMotor1;
    private final DcMotor SlideMotor2;

    public SlidePT(RobotParametersPT params, HardwareMap hardwareMap){
        SlideMotor1 = hardwareMap.get(DcMotor.class, params.slideMotorName1);
        SlideMotor2 = hardwareMap.get(DcMotor.class, params.slideMotorName2);
        SlideMotor2.setDirection(DcMotor.Direction.REVERSE);
        SlideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private RobotParametersPT params;

    public void stateUpdate(RobotParametersPT.SlideState slideState, double power) {
        switch(slideState){
            case SLIDE_IN:
                slideIn(power);
                break;

            case SLIDE_OUT:
                slideOut(-power);
                break;

            case STOP:
                stop();
                break;
        }
    }

    public void slideIn(double power){
        SlideMotor1.setPower(power);
      SlideMotor2.setPower(power);
    }

    public void slideOut(double power){
        SlideMotor1.setPower(power);
        SlideMotor2.setPower(power);
    }

    public void stop(){
        SlideMotor1.setPower(0);
        SlideMotor2.setPower(0);

    }


}
