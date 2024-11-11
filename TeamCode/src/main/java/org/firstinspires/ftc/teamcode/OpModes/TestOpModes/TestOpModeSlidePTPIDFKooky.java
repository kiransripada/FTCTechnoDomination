//Leilanie

package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

/****/
@Config
/****/

@TeleOp(name="SlideTestPTPIDFKooky", group="TestOpModes")
public class TestOpModeSlidePTPIDFKooky extends OpMode {

    private PIDController controller;
    public static double p =0.035,i=0.0,d=0.00075;
    public static double f=0.55;

    public static int target = 0;
    private final double ticks_in_degree = 384.5/180.0;
    private DcMotorEx SlideMotor1;
    private DcMotorEx SlideMotor2;
    boolean targetReached = false;

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        SlideMotor1 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.slideMotorName1);
        SlideMotor2 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.slideMotorName2);
        SlideMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        SlideMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        SlideMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop(){

            controller.setPID(p, i, d);

            int slidePos = SlideMotor1.getCurrentPosition();
            double pid = controller.calculate(slidePos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;


            SlideMotor1.setPower(Range.clip(power * .75,-0.75,0.75));
            SlideMotor2.setPower(Range.clip(power * .75,-0.75,0.75));

            if (SlideMotor1.getCurrent(CurrentUnit.AMPS) > 5){
                target = SlideMotor1.getCurrentPosition() + 50;
                SlideMotor1.setPower(0);
                SlideMotor2.setPower(0);
            }

            telemetry.addData("pos", slidePos);
            telemetry.addData("pid", pid);
            telemetry.addData("ff", ff);
            telemetry.addData("power ", power);
            telemetry.addData("target ", target);
            telemetry.addData("Current ",SlideMotor1.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
    }
}



