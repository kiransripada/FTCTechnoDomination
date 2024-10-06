package org.firstinspires.ftc.teamcode.OpModes.Auto;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

@Autonomous(name="AutoOpPT", group="Autonomous")
public class AutoOpPT extends LinearOpMode {

    private RobotParametersPT params;
    private Robot myRobot;
    private ElapsedTime runtime = new ElapsedTime();
    int phase = 0;
    //hello world; we are about to override
    @Override
    public void runOpMode(){
        params = new RobotParametersPT();
        myRobot = new Robot(params,hardwareMap,true,false,false, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Autonomous: Parking in observation zone
        telemetry.addData("Status", "Running");
        telemetry.update();

        runtime.reset();
        phase = 0;

       /*
       while (opModeIsActive()){
            myRobot.driveTrain.driveStraight(.5);
            sleep(2000);
            break;
        }
        */

        while (opModeIsActive()){

            myRobot.driveTrain.driveStraight(params.defaultDrivePower*params.powerReduction,5);
            while (myRobot.driveTrain.FrontLeftDCMotor.isBusy()){
            }
            myRobot.driveTrain.stop();
            sleep(2000);

            myRobot.driveTrain.driveStraight(-params.defaultDrivePower*params.powerReduction,-5);
            while (myRobot.driveTrain.FrontLeftDCMotor.isBusy()){
            }
            myRobot.driveTrain.stop();
            sleep(2000);

            myRobot.driveTrain.setWithoutEncoder();

            telemetry.addData("Yaw is 1: ",myRobot.driveTrain.getYaw());
            telemetry.update();
            sleep(2500);

            myRobot.driveTrain.turnRight(90);
            myRobot.driveTrain.alignAngle(90);

            telemetry.addData("Yaw is 2: ",myRobot.driveTrain.getYaw());
            telemetry.update();
            sleep(2500);

            myRobot.driveTrain.turnLeft(-90);
            myRobot.driveTrain.alignAngle(-90);

            telemetry.addData("Yaw is 3: ",myRobot.driveTrain.getYaw());
            telemetry.update();
            sleep(2500);

            myRobot.driveTrain.stop();
            sleep(2000);
            phase++;
            break;
        }

        myRobot.driveTrain.stop();

    }

}

