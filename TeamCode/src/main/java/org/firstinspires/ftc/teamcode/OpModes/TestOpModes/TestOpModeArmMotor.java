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
            telemetry.addData("Arm Position #1", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1));
            telemetry.addData("Arm Position #2", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor2));
            telemetry.update();
            //Move arm
            myRobot.arm.moveArm(-10);

            telemetry.addData("Arm Position #1", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1));
            telemetry.addData("Arm Position #2", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor2));
            telemetry.update();

            while (myRobot.arm.ArmMotor1.isBusy() || myRobot.arm.ArmMotor2.isBusy()) {
                telemetry.addData("Arm Position #1", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1));
                telemetry.addData("Arm Position #2", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor2));
                telemetry.update();
            }

            telemetry.addData("Arm Position #1", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1));
            telemetry.addData("Arm Position #2", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor2));
            telemetry.update();


        }

        if (gamepad2.b){
            myRobot.arm.stop();
        }

/*        if (gamepad2.dpad_down) {
            //Starting position
            telemetry.addData("Arm Position #1", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1));
            telemetry.update();
            telemetry.addData("Arm Position #2", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor2));
            telemetry.update();

            //Moving with power 0.1
            myRobot.arm.noEncoderMovement(0.1);

            //Ending position
            telemetry.addData("Arm Position #1", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1));
            telemetry.update();
            telemetry.addData("Arm Position #2", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor2));
            telemetry.update();

        }
*/

    }

}

