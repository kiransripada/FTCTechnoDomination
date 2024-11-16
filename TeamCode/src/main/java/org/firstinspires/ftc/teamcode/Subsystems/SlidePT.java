package org.firstinspires.ftc.teamcode.Subsystems;
//JJ
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;
public class SlidePT {
    private PIDController controller;
    public static double p =0.035,i=0.0,d=0.00075;
    public static double f=0.55;

    public static int target = 0;
    public static int slideStartingPosition;
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
        //SlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideStartingPosition = SlideMotor1.getCurrentPosition();

        SlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

        double power = pid + ff;


        SlideMotor1.setPower(Range.clip(power * .75,-0.75,0.75));
        SlideMotor2.setPower(Range.clip(power * .75,-0.75,0.75));

        if (SlideMotor1.getCurrent(CurrentUnit.AMPS) > 6 || SlideMotor2.getCurrent(CurrentUnit.AMPS) > 6){
            target = SlideMotor1.getCurrentPosition() + 50;
            SlideMotor1.setPower(0);
            SlideMotor2.setPower(0);
        }

    }
    public String getTelemetryForSlides(){

        String telemetry = "";
        telemetry = telemetry + "\n pos - " + SlideMotor1.getCurrentPosition();
        telemetry = telemetry + "\n pid - " + pid;
        telemetry = telemetry + "\n ff - " + ff;
        telemetry = telemetry + "\n power - " + power;
        telemetry = telemetry + "\n Current - " + SlideMotor1.getCurrent(CurrentUnit.AMPS);
        telemetry = telemetry + "\n target - " + target;
        telemetry = telemetry + "\n startingPos - " + slideStartingPosition;

        return telemetry;
    }


}
