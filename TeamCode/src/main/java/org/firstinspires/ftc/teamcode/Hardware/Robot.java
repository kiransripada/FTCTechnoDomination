//Leilanie

package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.ArmMotor;
import org.firstinspires.ftc.teamcode.Subsystems.ClawPT;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainPT;
import org.firstinspires.ftc.teamcode.Subsystems.SlidePT;

public class Robot {
    public RobotParametersPT params;
    public DriveTrainPT driveTrain;
    public SlidePT slide;
    public ClawPT claw;
    public ArmMotor arm;

    public Robot(RobotParametersPT params, HardwareMap hardwareMap, boolean isDriveTrain, boolean isSlide, boolean isClaw, boolean isArm){
        if (isDriveTrain)
            driveTrain = new DriveTrainPT(params,hardwareMap);
        if (isSlide)
            slide = new SlidePT(params, hardwareMap);
        if (isClaw)
            claw = new ClawPT(params, hardwareMap);
        if (isArm)
            arm = new ArmMotor(params, hardwareMap);
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


    public void slidePullIn(){

        slide.stateUpdate(RobotParametersPT.SlideState.SLIDE_IN, RobotParametersPT.defaultSlidePower);
    }

    public void slidePushOut(){

        slide.stateUpdate(RobotParametersPT.SlideState.SLIDE_OUT, RobotParametersPT.defaultSlidePower);
    }

    public void slideStop(){

        slide.stateUpdate(RobotParametersPT.SlideState.STOP, RobotParametersPT.defaultSlidePower);
    }

    public void clawTurnIn(){
        claw.stateUpdate(RobotParametersPT.ClawState.TURN_IN, RobotParametersPT.defaultClawPower);
    }

    public void clawTurnOut(){
        claw.stateUpdate(RobotParametersPT.ClawState.TURN_OUT, RobotParametersPT.defaultClawPower);
    }

    public void clawStop(){
        claw.stateUpdate(RobotParametersPT.ClawState.STOP, RobotParametersPT.defaultClawPower);
    }
    public void armPivotUp(){
        arm.stateUpdate(RobotParametersPT.ArmState.PIVOT_UP, RobotParametersPT.defaultArmPower);
    }

    public void armPivotDown(){
        arm.stateUpdate(RobotParametersPT.ArmState.PIVOT_DOWN, RobotParametersPT.defaultArmPower);
    }

    public void armStop(){
        arm.stateUpdate(RobotParametersPT.ArmState.STOP, RobotParametersPT.defaultArmPower);
    }


    public void stopAll(){
        driveTrain.stop();

    }

}
