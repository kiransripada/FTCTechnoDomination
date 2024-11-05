//Leilanie

package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

@Autonomous(name="AutoParkLeftPT", group="Autonomous")
public class AutoParkLeftPT extends LinearOpMode {


    private RobotParametersPT params;
    private Robot myRobot;
    private ElapsedTime runtime = new ElapsedTime();
    int phase = 0;

    boolean driveStraightReached = false;
    boolean stepOne = false;

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

            //Go forward 41 in
            if (!stepOne) {
                driveStraightReached = myRobot.driveTrain.driveStraightPT(params.defaultDrivePower * params.powerReduction, 25.0);
            }
             if (driveStraightReached) {
                 stepOne = true;
             }


            //break;

        }



    }

}

