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
        myRobot = new Robot(params,hardwareMap,false,false,true, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void loop(){

        //Slide control
        if (gamepad2.right_bumper) {
            myRobot.arm.pivotUp(0.7);
        } else if (gamepad2.left_bumper) {
            myRobot.arm.pivotDown(0.7);
        } else {
            myRobot.arm.stop();
        }

    }
}

