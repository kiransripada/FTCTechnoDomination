package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;


@TeleOp(name="TeleOpPT", group="TeleOp")
public class TeleOpPT extends OpMode {

    private RobotParametersPT params;
    private Robot myRobot;
    private int cnt = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        params = new RobotParametersPT();
        myRobot = new Robot(params, hardwareMap, true, true, true, true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        //telemetry.addData("Status", "In loop");
        // Drivetrain control
        //double drive = -gamepad1.left_stick_y * params.powerReduction;
        //double strafe = gamepad1.left_stick_x * params.powerReduction;
        //double rotate = gamepad1.right_stick_x * params.powerReduction;

        //myRobot.driveTrain.drive(drive, strafe, rotate);

        // Intake control
        if (gamepad2.left_bumper) {
            myRobot.claw.turnIn(1);
        } else if (gamepad2.right_bumper) {
            myRobot.claw.turnOut(1);
            //Slide control
        }

        if (gamepad1.right_bumper) {
            myRobot.slide.slideIn(0.5);
        } else if (gamepad1.left_bumper) {
            myRobot.slide.slideOut(-0.5);
        } else {
            myRobot.slideStop();
        }

        // Send telemetry data
       // telemetry.addData("Status", "Running");
       // telemetry.addData("Drive", "drive (%.2f), strafe (%.2f), rotate (%.2f)");
       // telemetry.addData("arm pos", myRobot.arm.getTelemetryForArm());
       // telemetry.update();


        if (gamepad2.y) {
            myRobot.arm.moveArmVersion2(-200);




            telemetry.addData("Arm telemetry", myRobot.arm.getTelemetryForArm());
            telemetry.update();


        }

    }
}