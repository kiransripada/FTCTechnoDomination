package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainPT;
import org.firstinspires.ftc.teamcode.Subsystems.IntakePT;
import org.firstinspires.ftc.teamcode.Subsystems.SlidePT;

public class Robot {
    public RobotParametersPT params;
    public DriveTrainPT driveTrain;
    public IntakePT intake;
    public SlidePT slide;

    public Robot(RobotParametersPT params, HardwareMap hardwareMap){
        driveTrain = new DriveTrainPT(params,hardwareMap);
        intake = new IntakePT(params,hardwareMap);
        slide = new SlidePT(params, hardwareMap);
    }

    public void teleopDrive(double drive, double strafe, double rotate){
        driveTrain.drive(drive,strafe,rotate);
    }

    public void driveStraight(double power){
        driveTrain.driveStraight(power);
    }

    public void turnLeft(double power){
        driveTrain.turnLeft(power);
    }

    public void turnRight(double power){
        driveTrain.turnRight(power);
    }

    public void stopDriving(){
        driveTrain.stop();
    }

    public void intakePullIn(){
        //intake.pullIn(0.5);
        intake.stateUpdate(RobotParametersPT.IntakeState.PULL_IN,params.defaultIntakePower);
    }

    public void intakePushOut(){
        //intake.pushOut(0.5);
        intake.stateUpdate(RobotParametersPT.IntakeState.PUSH_OUT,params.defaultIntakePower);
    }

    public void intakeStop(){
        //intake.stop();
        intake.stateUpdate(RobotParametersPT.IntakeState.STOP,params.defaultIntakePower);
    }

    public void slidePullIn(){
        //intake.pullIn(0.5);
        slide.stateUpdate(RobotParametersPT.SlideState.SLIDE_IN,params.defaultSlidePower);
    }

    public void slidePushOut(){
        //intake.pushOut(0.5);
        slide.stateUpdate(RobotParametersPT.SlideState.SLIDE_OUT,params.defaultSlidePower);
    }

    public void slideStop(){
        //intake.stop();
        slide.stateUpdate(RobotParametersPT.SlideState.STOP,params.defaultSlidePower);
    }

    public void stopAll(){
        driveTrain.stop();
        intake.stop();
    }

}
