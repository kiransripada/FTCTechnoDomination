package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

@TeleOp(name="TeleOpPT", group="TeleOp")
public class TeleOpPT extends OpMode {

    private RobotParametersPT params;
    private Robot myRobot;
    private int cnt = 0;

    @Override
    public void init(){
        params = new RobotParametersPT();
        myRobot = new Robot(params, hardwareMap,true,true,true, true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop(){
        // Drivetrain control
        double drive = -gamepad1.left_stick_y * params.powerReduction;
        double strafe = gamepad1.left_stick_x * params.powerReduction;
        double rotate = gamepad1.right_stick_x * params.powerReduction;

        myRobot.driveTrain.drive(drive,strafe,rotate);

        // Intake control
        if (gamepad2.left_bumper) {
            myRobot.claw.turnIn(1);
        } else if (gamepad2.right_bumper) {
            myRobot.claw.turnOut(1);
        } else {
            myRobot.clawStop();
        }
        //Slide control
        if (gamepad1.right_bumper) {
            myRobot.slidePullIn();
        } else if (gamepad1.left_bumper) {
            myRobot.slidePushOut();
        } else {
            myRobot.slideStop();
        }

        // Send telemetry data
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive", "drive (%.2f), strafe (%.2f), rotate (%.2f)", drive, strafe, rotate);
        telemetry.update();

        telemetry.addData("start 2", myRobot.arm.getTelemetry());
        telemetry.update();

        if (gamepad2.y) {
            telemetry.addData("start 1", myRobot.arm.getTelemetry());
            telemetry.update();
            myRobot.arm.moveArm(-125);
            while (myRobot.arm.ArmMotor1.isBusy()) {
                telemetry.addData("start 2", myRobot.arm.getTelemetry());
                telemetry.update();
            }

            telemetry.addData("start ", myRobot.arm.getTelemetry());
            telemetry.update();
        }

        if (gamepad2.b) {
            if (cnt == 0){
                myRobot.arm.ArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                myRobot.arm.ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cnt++;
            }
            telemetry.addData("start 1", myRobot.arm.getTelemetry());
            telemetry.update();
            myRobot.arm.moveArm(-25);
            while (myRobot.arm.ArmMotor1.isBusy()) {
                telemetry.addData("start 2", myRobot.arm.getTelemetry());
                telemetry.update();
            }

            telemetry.addData("start ", myRobot.arm.getTelemetry());
            telemetry.update();
        }
        if (gamepad2.a) {
            telemetry.addData("start 1", myRobot.arm.getTelemetry());
            telemetry.update();
            myRobot.arm.moveArm(-5);
            while (myRobot.arm.ArmMotor1.isBusy()) {
                telemetry.addData("start 2", myRobot.arm.getTelemetry());
                telemetry.update();
            }

            telemetry.addData("start ", myRobot.arm.getTelemetry());
            telemetry.update();
        }
        if (gamepad2.x) {
            telemetry.addData("start 1", myRobot.arm.getTelemetry());
            telemetry.update();
            myRobot.arm.moveArm(-15);
            while (myRobot.arm.ArmMotor1.isBusy()) {
                telemetry.addData("start 2", myRobot.arm.getTelemetry());
                telemetry.update();
            }

            telemetry.addData("start ", myRobot.arm.getTelemetry());
            telemetry.update();
        }

    }
}
