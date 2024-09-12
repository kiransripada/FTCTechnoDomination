package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

public class DriveTrainPT {
    private RobotParametersPT params;
    private DcMotor FrontLeftDCMotor;
    private DcMotor FrontRightDCMotor;
    private DcMotor BackLeftDCMotor;
    private DcMotor BackRightDCMotor;

    public DriveTrainPT(RobotParametersPT params, HardwareMap hardwareMap){
        this.params = params;
        // Initialize drive motors
        FrontLeftDCMotor = hardwareMap.get(DcMotor.class, params.frontLeftMotorName);
        FrontRightDCMotor = hardwareMap.get(DcMotor.class, params.frontRightMotorName);
        BackLeftDCMotor = hardwareMap.get(DcMotor.class, params.backLeftMotorName);
        BackRightDCMotor = hardwareMap.get(DcMotor.class, params.backRightMotorName);

        // Set motor directions
        FrontLeftDCMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDCMotor.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDCMotor.setDirection(DcMotor.Direction.FORWARD);
        BackRightDCMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake when power is zero
        FrontLeftDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double drive, double strafe, double rotate) {
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        FrontLeftDCMotor.setPower(frontLeftPower);
        FrontRightDCMotor.setPower(frontRightPower);
        BackLeftDCMotor.setPower(backLeftPower);
        BackRightDCMotor.setPower(backRightPower);
    }

    public void driveStraight(double power) {
        FrontLeftDCMotor.setPower(power);
        FrontRightDCMotor.setPower(power);
        BackLeftDCMotor.setPower(power);
        BackRightDCMotor.setPower(power);
    }

    public void turnLeft(double power) {
        FrontLeftDCMotor.setPower(-power);
        FrontRightDCMotor.setPower(power);
        BackLeftDCMotor.setPower(-power);
        BackRightDCMotor.setPower(power);
    }

    public void turnRight(double power) {
        FrontLeftDCMotor.setPower(power);
        FrontRightDCMotor.setPower(-power);
        BackLeftDCMotor.setPower(power);
        BackRightDCMotor.setPower(-power);
    }

    public void stop() {
        FrontLeftDCMotor.setPower(0);
        FrontRightDCMotor.setPower(0);
        BackLeftDCMotor.setPower(0);
        BackRightDCMotor.setPower(0);
    }

}
