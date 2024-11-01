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

        if (gamepad2.y) {
            telemetry.addData("start 1", myRobot.arm.getTelemetry());
            telemetry.update();
            myRobot.arm.moveArm(-125);
            while (myRobot.arm.ArmMotor1.isBusy()) {
                telemetry.addData("start 2", myRobot.arm.getTelemetry());
                telemetry.update();
            }

            telemetry.addData("start ", myRobot.arm.getTelemetry());
            telemetry.update();
        }

        if (gamepad2.a) {
            telemetry.addData("start 1", myRobot.arm.getTelemetry());
            telemetry.update();
            myRobot.arm.moveArm(-25);
            while (myRobot.arm.ArmMotor1.isBusy()) {
                telemetry.addData("start 2", myRobot.arm.getTelemetry());
                telemetry.update();
            }

            telemetry.addData("start ", myRobot.arm.getTelemetry());
            telemetry.update();
        }

    }

}

