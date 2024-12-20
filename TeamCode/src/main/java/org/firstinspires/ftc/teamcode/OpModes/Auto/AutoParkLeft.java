//Leilanie

package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

@Autonomous(name="AutoParkLeft", group="Autonomous")
public class AutoParkLeft extends LinearOpMode {


    private RobotParametersPT params;
    private Robot myRobot;
    private ElapsedTime runtime = new ElapsedTime();
    int phase = 0;

    //hello world; we are about to override
    @Override
    public void runOpMode(){
        params = new RobotParametersPT();
        myRobot = new Robot(params,hardwareMap,true,true, true, true);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("yaw ",myRobot.driveTrain.getYaw());
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Autonomous: Parking in observation zone
        telemetry.addData("Status", "Running");
        telemetry.update();

        runtime.reset();
        phase = 0;




        while (opModeIsActive()){
            //myRobot.driveStraight(params.defaultDrivePower*params.powerReduction);
            //sleep(1500);
            //Go forward 41 in
            myRobot.driveTrain.driveStraight(params.defaultDrivePower*params.powerReduction, 42.0);
            while (myRobot.driveTrain.FrontLeftDCMotor.isBusy()) {}
            myRobot.driveTrain.stop();


            //Turn right to angle 90, using gyro
            myRobot.driveTrain.turnRightByGyro(-90, params.defaultDrivePower*params.powerReduction);
            while (myRobot.driveTrain.FrontLeftDCMotor.isBusy()) {
                telemetry.addData("yaw 1",myRobot.driveTrain.getYaw());
                telemetry.update();

            }
            myRobot.driveTrain.stop();


            //Go straight just a little bit to make sure its in the space
            myRobot.driveTrain.driveStraight(params.defaultDrivePower*params.powerReduction, 8.0);
            while (myRobot.driveTrain.FrontLeftDCMotor.isBusy()) {}
            myRobot.driveTrain.stop();


            telemetry.addData("Arm Position #1", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1));
            telemetry.update();

            myRobot.arm.moveArm(-350);
            while (myRobot.arm.ArmMotor1.isBusy()) {
                telemetry.addData("Arm Position #1", myRobot.arm.getCurrentPosition((myRobot.arm.ArmMotor1)));
                telemetry.update();
            }

            myRobot.driveTrain.stop();
            sleep(1000);

            telemetry.addData("Arm Position #1", myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1));
            telemetry.update();

            myRobot.arm.endAutoArmPosition = myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1);

            break;

        }



    }

}

