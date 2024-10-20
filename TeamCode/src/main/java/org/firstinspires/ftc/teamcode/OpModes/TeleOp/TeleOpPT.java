package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;
import org.firstinspires.ftc.teamcode.Subsystems.ArmMotor;

@TeleOp(name="TeleOpPT", group="TeleOp")
public class TeleOpPT extends OpMode {

    private RobotParametersPT params;
    private Robot myRobot;

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

        myRobot.teleopDrive(drive,strafe,rotate);

        // clawl control
        if (gamepad2.dpad_left) {
            myRobot.claw.turnIn(1);
        } else if (gamepad2.b) {
            myRobot.claw.turnOut(1);
        } else {
            myRobot.clawStop();
        }
        //Slide control
        if (gamepad2.right_bumper) {
            myRobot.slidePullIn();
        } else if (gamepad2.left_bumper) {
            myRobot.slidePushOut();
        } else {
            myRobot.slideStop();
        }
        //Arm control
        if (gamepad1.right_bumper) {
            myRobot.arm.pivotUp(.75);
        } else if (gamepad1.left_bumper) {
            myRobot.arm.pivotDown(.75);
        } else {
            myRobot.arm.stop();
        }

        // Send telemetry data
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive", "drive (%.2f), strafe (%.2f), rotate (%.2f)", drive, strafe, rotate);
        telemetry.update();

    }
}
