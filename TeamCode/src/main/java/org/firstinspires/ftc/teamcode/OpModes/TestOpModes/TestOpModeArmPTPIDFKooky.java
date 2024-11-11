//Leilanie

package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

/****/
@Config
/****/

@TeleOp(name="ArmTestPTPIDFKooky", group="TestOpModes")
public class TestOpModeArmPTPIDFKooky extends OpMode {

    private PIDController controller;
    public static double p =0.0180,i=0,d=0.0009;
    public static double f=0.77;

    public static int target = 0;
    private final double ticks_in_degree = 1425.1/180.0;
    private DcMotorEx ArmMotor1;
    boolean targetReached = false;

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        ArmMotor1 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.armMotorName1);
        //ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        ArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop(){

            controller.setPID(p, i, d);

            int armPos = ArmMotor1.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            //ArmMotor1.setPower(power * .75);

            ArmMotor1.setPower(Range.clip(power * .75,-0.5,0.5));

            telemetry.addData("pos", armPos);
            telemetry.addData("pid", pid);
            telemetry.addData("ff", ff);
            telemetry.addData("power ", power);
            telemetry.addData("target ", target);
            telemetry.update();
    }
}



