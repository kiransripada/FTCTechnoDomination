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

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

/****/
@Config
/****/

@TeleOp(name="ArmTestPTPIDFCoeff", group="TestOpModes")
public class TestOpModeArmPTPIDFCoeff extends OpMode {

    private RobotParametersPT params;
    private Robot myRobot;

    /****/
    public static double P = 0.0, I =0.0, D=0.0, F=0.0;
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
        telemetry.addData("Position", ArmMotor1.getCurrentPosition());
        telemetry.update();
        pidfNew = new PIDFCoefficients(P,I,D,F);
       ArmMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

       if (gamepad2.y) {
           ArmMotor1.setTargetPosition((int) -175);
           ArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           ArmMotor1.setPower(0.5);
       }
        if (gamepad2.a) {
            ArmMotor1.setTargetPosition((int) -250);
            ArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmMotor1.setPower(0.5);
        }

        if (gamepad2.b) {
            ArmMotor1.setTargetPosition((int) 0);
            ArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmMotor1.setPower(0.5);
        }

    }

}


