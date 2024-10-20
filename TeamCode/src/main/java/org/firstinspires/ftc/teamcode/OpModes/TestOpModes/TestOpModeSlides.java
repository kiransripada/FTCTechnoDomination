package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;
//JJ
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

@TeleOp(name="SlidesTest", group="TestOpModes")
public class TestOpModeSlides extends OpMode {

    private RobotParametersPT params;
    private Robot myRobot;

    @Override
    public void init(){
        params = new RobotParametersPT();
        myRobot = new Robot(params,hardwareMap,false,true,false, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
//hello
    //this is prem hello
    @Override
    public void loop(){

        //Slide control
        if (gamepad2.right_bumper) {
            myRobot.slidePullIn();
        } else if (gamepad2.left_bumper) {
            myRobot.slidePushOut();
        } else {
            myRobot.slideStop();
        }

    }
}

