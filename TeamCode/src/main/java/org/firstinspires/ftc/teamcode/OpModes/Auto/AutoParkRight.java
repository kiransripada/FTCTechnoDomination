package org.firstinspires.ftc.teamcode.OpModes.Auto;
//JJ

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;
import org.firstinspires.ftc.teamcode.Subsystems.ArmMotor;
@Autonomous(name="AutoParkRight", group="Autonomous")
public class AutoParkRight extends LinearOpMode {


    private RobotParametersPT params;
    private Robot myRobot;
    private ElapsedTime runtime = new ElapsedTime();
    int phase = 0;
//hello world; we are about to override
@Override
public void runOpMode(){
    params = new RobotParametersPT();
    myRobot = new Robot(params,hardwareMap,true,false,true, true);
    telemetry.addData("Status", "Initialized");
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
        myRobot.driveTrain.driveStraight(params.defaultDrivePower*params.powerReduction, 24);
        while (myRobot.driveTrain.FrontLeftDCMotor.isBusy()) {}
        myRobot.driveTrain.driveStraight(params.defaultDrivePower*params.powerReduction, 20);
        while (myRobot.driveTrain.FrontLeftDCMotor.isBusy()) {}
        myRobot.driveTrain.turnRightByGyro(90,20);
        while (myRobot.driveTrain.FrontLeftDCMotor.isBusy()) {}
        myRobot.driveTrain.driveStraight(params.defaultDrivePower*params.powerReduction, 44.0);
        while (myRobot.driveTrain.FrontLeftDCMotor.isBusy()) {}
        myRobot.driveTrain.stop();
        break;
    }

}

   }

