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

    @Override
    public void runOpMode(){
        params = new RobotParametersPT();
        myRobot = new Robot(params,hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Autonomous routine: Drive forward for 5 seconds
        telemetry.addData("Status", "Running");
        telemetry.update();

        runtime.reset();
        phase = 0;

        while (opModeIsActive()){
            switch (phase){
                case 0:
                    myRobot.driveStraight(params.defaultDrivePower*params.powerReduction);
                    sleep(500);
                    phase++;
                    break;

                case 1:
                    myRobot.turnLeft(params.defaultDrivePower*params.powerReduction);
                    sleep(500);
                    phase++;
                    break;

                case 2:
                    myRobot.stopDriving();
                    myRobot.intakePullIn();
                    sleep(3000);
                    phase++;
                    break;

                case 3:
                    myRobot.stopAll();
                    break;
            }

        }

   }
}
