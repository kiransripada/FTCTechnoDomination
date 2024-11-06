package org.firstinspires.ftc.teamcode.Subsystems;
//JJ
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;
public class SlidePT {
    private PIDController controller;
    public static double p =0.0180,i=0,d=0.0009;
    public static double f=0.77;

    public static int target = 0;
    private final double ticks_in_degree = 384.5/180.0;
    public DcMotorEx SlideMotor1;
    public DcMotorEx SlideMotor2;
    boolean targetReached = false;

    int slidePos;
    double pid;
    double ff;
    double power;


    public SlidePT(RobotParametersPT params, HardwareMap hardwareMap){

        controller = new PIDController(p,i,d);

        SlideMotor1 = hardwareMap.get(DcMotorEx.class, params.slideMotorName1);
        SlideMotor2 = hardwareMap.get(DcMotorEx.class, params.slideMotorName2);
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

    public void moveSlidesVersion2 (int target) {
        controller.setPID(p, i, d);

        int slidePos = SlideMotor1.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid;

        SlideMotor1.setPower(power * .75);
        SlideMotor2.setPower(power * .75);
    }
    public String getTelemetryForSlides(){

        String telemetry = "";
        telemetry = telemetry + "pos - " + SlideMotor1.getCurrentPosition();
        telemetry = telemetry + "pid"+ pid;
        telemetry = telemetry + "ff"+ ff;
        telemetry = telemetry + "power"+ power;
        telemetry = telemetry + "target"+ target;

        return telemetry;
    }


}
