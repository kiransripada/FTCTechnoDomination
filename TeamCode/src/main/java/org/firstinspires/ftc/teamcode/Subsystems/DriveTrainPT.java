package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrainPT {
    private RobotParametersPT params;
    public DcMotor FrontLeftDCMotor;
    public DcMotor FrontRightDCMotor;
    public DcMotor BackLeftDCMotor;
    public DcMotor BackRightDCMotor;
    private IMU imu;
    private YawPitchRollAngles orientation;

    private void setWithEncoder(){
        // Set motor encoder mode
        FrontLeftDCMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDCMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRightDCMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDCMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setWithoutEncoder(){
        // Set motor encoder mode
        FrontLeftDCMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontRightDCMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }



    public DriveTrainPT(RobotParametersPT params, HardwareMap hardwareMap){
        this.params = params;
        // Initialize drive motors
        FrontLeftDCMotor = hardwareMap.get(DcMotor.class, params.frontLeftMotorName);
        FrontRightDCMotor = hardwareMap.get(DcMotor.class, params.frontRightMotorName);
        BackLeftDCMotor = hardwareMap.get(DcMotor.class, params.backLeftMotorName);
        BackRightDCMotor = hardwareMap.get(DcMotor.class, params.backRightMotorName);

        // Set motor directions
        FrontLeftDCMotor.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDCMotor.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDCMotor.setDirection(DcMotor.Direction.REVERSE);
        BackRightDCMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motor encoder mode
        setWithEncoder();

        // Set all motors to brake when power is zero
        FrontLeftDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu.resetYaw();
    }

    public void drive(double drive, double strafe, double rotate) {
        setWithoutEncoder();

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate),1);

        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        FrontLeftDCMotor.setPower(frontLeftPower/denominator);
        FrontRightDCMotor.setPower(frontRightPower/denominator);
        BackLeftDCMotor.setPower(backLeftPower/denominator);
        BackRightDCMotor.setPower(backRightPower/denominator);
    }

    public void driveStraight(double power) {

        setWithoutEncoder();

        FrontLeftDCMotor.setPower(power);
        FrontRightDCMotor.setPower(power);
        BackLeftDCMotor.setPower(power);
        BackRightDCMotor.setPower(power);
    }

    public int getNewPosition(double distance){
        double Counts_Per_Motor_Rev = params.Counts_Per_Motor_Rev;
        double Drive_Gear_Reduction = params.Drive_Gear_Reduction;
        double Wheel_Diameter = params.Wheel_Diameter;
        double Counts_Per_Inch = ( Counts_Per_Motor_Rev * Drive_Gear_Reduction) / (Wheel_Diameter * 3.1415);
        return (int)(distance * Counts_Per_Inch);
    }

    public double getYaw(){
        orientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = orientation.getYaw(AngleUnit.DEGREES);

        return currentYaw;

    }

    public void driveStraight(double power, double distance) {

        setWithEncoder();

        int newLeftTarget = FrontLeftDCMotor.getCurrentPosition() + (int)(getNewPosition(distance));
        int newRightTarget = FrontRightDCMotor.getCurrentPosition() + (int)(getNewPosition(distance));


        FrontLeftDCMotor.setTargetPosition(newLeftTarget);
        FrontRightDCMotor.setTargetPosition(newRightTarget);
        FrontLeftDCMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDCMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDCMotor.setPower(power);
        FrontRightDCMotor.setPower(power);
        BackLeftDCMotor.setPower(power);
        BackRightDCMotor.setPower(power);


    }

    public void turnLeft( double angle) {
        setWithoutEncoder();
        while (angle > getYaw()) {
            FrontLeftDCMotor.setPower(-params.defaultTurnPower*params.powerReduction);
            FrontRightDCMotor.setPower(params.defaultTurnPower);
            BackLeftDCMotor.setPower(-params.defaultTurnPower*params.powerReduction);
            BackRightDCMotor.setPower(params.defaultTurnPower);
        }
        stop();
    }

    public void turnRight( double angle) {
        setWithoutEncoder();
        while (angle < getYaw()) {
            FrontLeftDCMotor.setPower(params.defaultTurnPower);
            FrontRightDCMotor.setPower(-params.defaultTurnPower*params.powerReduction);
            BackLeftDCMotor.setPower(params.defaultTurnPower);
            BackRightDCMotor.setPower(-params.defaultTurnPower*params.powerReduction);
        }
        stop();
    }

    public void alignAngle(double angle){
        setWithoutEncoder();
        if (angle > getYaw()){
            turnLeft(angle);
        }
        else {
            turnRight(angle);
        }
    }

    public void stop() {
        FrontLeftDCMotor.setPower(0);
        FrontRightDCMotor.setPower(0);
        BackLeftDCMotor.setPower(0);
        BackRightDCMotor.setPower(0);
    }



}
