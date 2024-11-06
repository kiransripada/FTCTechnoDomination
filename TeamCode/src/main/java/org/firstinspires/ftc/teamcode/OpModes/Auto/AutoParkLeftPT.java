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
    boolean stepOne = true;
    boolean turnReached = false;
    boolean stepTwo = false;
    boolean armRaised = false;
    boolean stepThree = false;
    boolean slideRaised = false;
    boolean stepFour = false;

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
            if (stepOne) {
                myRobot.driveTrain.initializedFrontLeft = false;
                myRobot.driveTrain.initializedFrontRight = false;
                telemetry.addData("StepOne", " tar pos " + myRobot.driveTrain.getNewPosition(25.0));
                telemetry.update();
                sleep(3000);
                driveStraightReached = myRobot.driveTrain.driveStraightPT(params.defaultDrivePower * params.powerReduction, 15.0);
                telemetry.addData("StepOne", "Running curr pos " + myRobot.driveTrain.FrontLeftDCMotor.getCurrentPosition());
                telemetry.addData("StepOne", "drive reached " + driveStraightReached);
                telemetry.update();
                sleep(2000);
            }
             if (driveStraightReached) {
                 stepOne = false;
                 driveStraightReached = false;
                 telemetry.addData("StepOne after", "drive reached " + driveStraightReached);
                 telemetry.update();
                 myRobot.driveTrain.stop();
                 stepTwo = true;
             }

             if (stepTwo){
                 turnReached = myRobot.driveTrain.turnRightByGyroPT(-90, params.defaultDrivePower*params.powerReduction);
                 telemetry.addData("StepTwo", "current yaw " + myRobot.driveTrain.getYaw());
                 telemetry.addData("StepTwo", "turn reached " + turnReached);
                 telemetry.update();
             }

             if (turnReached) {
                 stepTwo = false;
                 turnReached = false;
                 myRobot.driveTrain.stop();
                 stepThree = true;
             }

            if (stepThree) {
                myRobot.arm.moveArmVersion2(-350);
                telemetry.addData("Arm telemetry", myRobot.arm.getTelemetryForArm());
                telemetry.update();

                if (myRobot.arm.ArmMotor1.getCurrentPosition() >= -375 && myRobot.arm.ArmMotor1.getCurrentPosition() <= -325){
                    armRaised = true;
                }
            }

            if (armRaised) {
                stepThree = true;
                armRaised = false;
                stepFour = true;
            }

            if (stepFour) {
                myRobot.slide.moveSlidesVersion2(400);
                telemetry.addData("Slides telemetry", myRobot.slide.getTelemetryForSlides());
                telemetry.update();

                if (myRobot.slide.SlideMotor1.getCurrentPosition() >= 375 && myRobot.slide.SlideMotor1.getCurrentPosition() <= 425){
                    slideRaised = true;
                }

            }

            if (slideRaised) {
                stepFour = true;
                slideRaised = false;
            }


            //break;

        }



    }

}

