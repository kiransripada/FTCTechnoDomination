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

    boolean driveToBasket = false;
    double driveToBasketDis = 30.0;
    boolean driveBackToStart = false;
    double driveBackToStartDis = 5.0;
    int startPosTracking = 0;
    boolean stepOne = true;
    boolean turnReached = false;
    boolean stepTwo = false;
    boolean armRaised = false;
    boolean stepThree = false;
    boolean slideRaised = false;
    boolean stepFour = false;
    boolean stepFive = false;
    boolean driveStarted = false;
    boolean startTimerStepOne = false;
    boolean startTimerStepTwo = false;

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


        startPosTracking = myRobot.driveTrain.FrontLeftDCMotor.getCurrentPosition();

        while (opModeIsActive()){

            if (stepOne) {
                myRobot.driveTrain.initializedFrontLeft = false;
                myRobot.driveTrain.initializedFrontRight = false;

                driveToBasket = myRobot.driveTrain.driveStraightPT(params.defaultDrivePower * params.powerReduction, driveToBasketDis);

                telemetry.addData("StepOne FrontLeft Curr Position - ",myRobot.driveTrain.FrontLeftDCMotor.getCurrentPosition());
                telemetry.addData("StepOne Get Drive New Position - ",myRobot.driveTrain.getNewPosition(driveToBasketDis));
                telemetry.update();

                if ((myRobot.driveTrain.FrontLeftDCMotor.getCurrentPosition() - myRobot.driveTrain.getNewPosition(driveToBasketDis)) > 20) {
                    driveToBasket = true;
                } else {
                    driveToBasket = false;
                }
                telemetry.addData("driveToBasket ", driveToBasket);
                telemetry.update();
                if (driveToBasket) {

                    myRobot.driveTrain.stop();
                    stepOne = false;
                   // stepTwo = true;
                    runtime.reset();
                    startTimerStepOne = true;
                    startPosTracking = myRobot.driveTrain.FrontLeftDCMotor.getCurrentPosition();
                }



            }

            if( startTimerStepOne){
                if (runtime.seconds() > 5){
                    stepTwo = true;
                    stepOne = false;
                    startTimerStepOne = false;
                }
            }

            if (stepTwo) {

                myRobot.driveTrain.initializedFrontLeft = false;
                myRobot.driveTrain.initializedFrontRight = false;
                if (!driveStarted) {
                    driveBackToStart = myRobot.driveTrain.driveBackPT(params.defaultDrivePower * params.powerReduction, driveBackToStartDis);
                    driveStarted = true;
                }
                telemetry.addData("StepTwo FrontLeft Curr Position - ",myRobot.driveTrain.FrontLeftDCMotor.getCurrentPosition());
                telemetry.addData("Start Pos tracking ",startPosTracking);
                telemetry.addData("StepTwo Get Drive New Position - ",myRobot.driveTrain.getNewPosition(driveBackToStartDis));
                telemetry.update();

                if ((myRobot.driveTrain.FrontLeftDCMotor.getCurrentPosition()-100) < (startPosTracking - myRobot.driveTrain.getNewPosition(driveBackToStartDis))) {
                    driveBackToStart = true;
                }
                else {
                    driveBackToStart = false;

                }



                /*if ((myRobot.driveTrain.getNewPosition(driveBackToStartDis) - myRobot.driveTrain.FrontLeftDCMotor.getCurrentPosition()) > 20) {
                    driveBackToStart = true;
                } else {
                    driveBackToStart = false;
                }*/
                telemetry.addData("driveBackToStart ", driveBackToStart);
                telemetry.update();
                if (driveBackToStart) {
                    stepTwo = false;
                    myRobot.driveTrain.stop();

                    stepThree = true;

                    runtime.reset();
                    startTimerStepTwo = true;
                   // break;
                }


            }

            if( startTimerStepTwo){
                if (runtime.seconds() > 5){
                    stepThree = true;
                    stepTwo = false;
                    startTimerStepTwo = false;
                    break;
                }
            }



        }



    }

}

