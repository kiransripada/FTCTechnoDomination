package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

@TeleOp(name="DrivetrainTest", group="TestOpModes")
public class TestOpModeDriveTrain extends OpMode {

    private RobotParametersPT params;
    private Robot myRobot;

    @Override
    public void init(){
        params = new RobotParametersPT();
        myRobot = new Robot(params,hardwareMap,true,false,false);
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


        // Send telemetry data
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive", "drive (%.2f), strafe (%.2f), rotate (%.2f)", drive, strafe, rotate);
        telemetry.update();

    }
}
