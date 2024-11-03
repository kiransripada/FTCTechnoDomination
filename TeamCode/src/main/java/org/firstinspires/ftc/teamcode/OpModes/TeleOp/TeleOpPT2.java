package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;


@TeleOp(name="TeleOpPT2", group="TeleOp")
public class TeleOpPT2 extends OpMode {

    private RobotParametersPT params;
    private Robot myRobot;
    private int cnt = 0;

    private PIDController controller;
    public static double p =0.0180,i=0,d=0.0009;
    public static double f=0.77;

    public static int target = 0;
    private final double ticks_in_degree = 1425.1/180.0;
    private DcMotorEx ArmMotor1;



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        params = new RobotParametersPT();
        myRobot = new Robot(params, hardwareMap, true, true, true, true);

        controller = new PIDController(p,i,d);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);

        int armPos = ArmMotor1.getCurrentPosition();
        double pid = controller.calculate(armPos, -200);
        double ff = Math.cos(Math.toRadians(-200 / ticks_in_degree)) * f;

        double power = pid + ff;

        ArmMotor1.setPower(power * .75);

        telemetry.addData("pos", armPos);
        telemetry.addData("pid", pid);
        telemetry.addData("ff", ff);
        telemetry.addData("power ", power);
        telemetry.addData("target ", target);
        telemetry.update();

    }
}
