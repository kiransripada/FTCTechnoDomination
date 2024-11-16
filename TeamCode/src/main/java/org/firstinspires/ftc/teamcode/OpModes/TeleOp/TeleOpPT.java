package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

@TeleOp(name="TeleOpPT", group="TeleOp")
public class TeleOpPT extends OpMode {
    //hi

    private RobotParametersPT params;
    private Robot myRobot;
    private int cnt = 0;
    private int armTargetPos = 0;
    private int slideTargetPos = 0;
    boolean pressedOrNotPressedArm = false;
    boolean pressedOrNotPressedSlide = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        params = new RobotParametersPT();
        myRobot = new Robot(params, hardwareMap, true, true, true, true);

        //myRobot.arm.endAutoArmPosition = myRobot.arm.getCurrentPosition(myRobot.arm.ArmMotor1);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Auto Arm Position",myRobot.arm.endAutoArmPosition);
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "In loop");
        // Drivetrain control
        double drive = -gamepad1.left_stick_y * params.powerReduction;
        double strafe = gamepad1.left_stick_x * params.powerReduction;
        double rotate = gamepad1.right_stick_x * params.powerReduction;

        myRobot.driveTrain.drive(drive, strafe, rotate);

        // Claw control
        if (gamepad2.left_bumper) {
            myRobot.claw.turnIn(1);
        } else if (gamepad2.right_bumper) {
            if (myRobot.slide.SlideMotor1.getCurrentPosition() > 1500) {
                myRobot.arm.moveArmVersion2(-400);
                myRobot.claw.turnOut(1);
                myRobot.arm.moveArmVersion2(-350);
                pressedOrNotPressedArm = true;
            } else {
                myRobot.claw.turnOut(1);
            }
        }

        telemetry.addData("Slide Current Position" , myRobot.slide.SlideMotor1.getCurrentPosition());

        //Slide controls
        if (gamepad1.y) {
            slideTargetPos = myRobot.slide.slideStartingPosition + 2100;
            pressedOrNotPressedSlide = true;
        } else if (gamepad1.a) {
            slideTargetPos = myRobot.slide.slideStartingPosition;
            pressedOrNotPressedSlide = true;
        } else if (gamepad1.b){
            slideTargetPos = myRobot.slide.slideStartingPosition+ 750;
            pressedOrNotPressedSlide = true;
        }

        if (pressedOrNotPressedSlide == true) {
            myRobot.slide.moveSlidesVersion2(slideTargetPos);
            telemetry.addData("Slides telemetry", myRobot.slide.getTelemetryForSlides());
            telemetry.update();
        }

        // Send telemetry data
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive", "drive (%.2f), strafe (%.2f), rotate (%.2f)");
        telemetry.addData("arm pos", myRobot.arm.getTelemetryForArm());
        telemetry.update();

        //Arm controls
        //Straight vertical
        if (gamepad2.y) {
            armTargetPos = -350;
            pressedOrNotPressedArm = true;
        //To pick sample off ground
        } else if (gamepad2.a) {
            if (myRobot.slide.SlideMotor1.getCurrentPosition() < 1000) {
                myRobot.claw.turnOut(1);
                armTargetPos = -740;
                pressedOrNotPressedArm = true;
            }
        //To pick inside submersible
        } else if (gamepad2.b) {
            armTargetPos = -600;
            pressedOrNotPressedArm = true;
        //To pick specimen
        } else if (gamepad2.x) {
            armTargetPos = -700;
            pressedOrNotPressedArm = true;
        }

        if (pressedOrNotPressedArm == true) {
            myRobot.arm.moveArmVersion2(armTargetPos);
            telemetry.addData("Arm telemetry", myRobot.arm.getTelemetryForArm());
            telemetry.update();
        }
    }
}