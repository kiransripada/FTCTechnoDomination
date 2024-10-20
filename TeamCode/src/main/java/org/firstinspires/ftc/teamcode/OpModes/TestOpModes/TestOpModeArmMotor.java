//Leilanie

package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

@TeleOp(name="ArmTest", group="TestOpModes")
public class TestOpModeArmMotor extends OpMode {

    private RobotParametersPT params;
    private Robot myRobot;

    @Override
    public void init(){
        params = new RobotParametersPT();
        myRobot = new Robot(params,hardwareMap,false,false,false, true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void loop(){



        //Slide control
        if (gamepad2.a) {
            telemetry.addData("Arm Position", myRobot.arm.getCurrentPosition());
            telemetry.update();

            myRobot.arm.moveArm(125);
            telemetry.addData("Arm Position", myRobot.arm.getCurrentPosition());
            telemetry.update();
            while (myRobot.arm.ArmMotor.isBusy()) {
                telemetry.addData("Arm Position", myRobot.arm.getCurrentPosition());
                telemetry.update();

            }
            myRobot.arm.stop();

            telemetry.addData("Arm Position", myRobot.arm.getCurrentPosition());
            telemetry.update();


        }

        if (gamepad2.b){
            myRobot.arm.stop();
        }


    }

}

