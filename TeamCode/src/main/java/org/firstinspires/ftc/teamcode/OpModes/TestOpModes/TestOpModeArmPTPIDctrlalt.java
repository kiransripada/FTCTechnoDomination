//Leilanie

package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

/****/
@Config
/****/

@TeleOp(name="ArmTestPTPIDctrlalt", group="TestOpModes")
public class TestOpModeArmPTPIDctrlalt extends OpMode {

    private RobotParametersPT params;
    private Robot myRobot;

    /****/
    public static double Kp = 0.0, Ki =0.0, Kd=0.0;
    public static double integralSum = 0.0, derivative = 0.0, power =0.0;
    public int lastError = 0, error = 0, encoderPosition = 0, targetPosition = 0;
    boolean targetReached = false;
    ElapsedTime timer = new ElapsedTime();
    public DcMotorEx ArmMotor1;
    PIDFCoefficients pidfNew = new PIDFCoefficients();
    /****/

    @Override
    public void init(){

        /****/
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ArmMotor1 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.armMotorName1);
        ArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ArmMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /****/

        params = new RobotParametersPT();
        myRobot = new Robot(params,hardwareMap,false,false,false, true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();



    }
    @Override
    public void loop(){
        telemetry.addData("Start pos", ArmMotor1.getCurrentPosition());
        telemetry.update();
        if (gamepad2.y){
            targetPosition = -125;
            telemetry.addData("Inside if","");
            telemetry.update();
            while (!targetReached){
                encoderPosition = ArmMotor1.getCurrentPosition();
                error = targetPosition - encoderPosition;
                derivative = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
                power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
                ArmMotor1.setPower(power);
                lastError = error;

                telemetry.addData("Inside position", ArmMotor1.getCurrentPosition());
                telemetry.addData("Inside error", error);
                telemetry.addData("Inside power", power);
                telemetry.update();

                if ((ArmMotor1.getCurrentPosition() <= targetPosition + 5) ||(ArmMotor1.getCurrentPosition() > targetPosition - 5) ) {
                    targetReached = true;
                    timer.reset();
                }

            }
        }

    }

}


