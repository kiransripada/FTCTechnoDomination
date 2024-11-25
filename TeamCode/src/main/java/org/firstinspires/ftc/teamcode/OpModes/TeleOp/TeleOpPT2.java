package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private DistanceSensor distance;
    private double distanceInch = 0.0;
    private double powerRange = 0.5;

    private boolean samplePick = false;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        ArmMotor1 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.armMotorName1);
        ArmMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        ArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        distance = hardwareMap.get(DistanceSensor.class,"distanceSensor");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distance;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        distanceInch = distance.getDistance(DistanceUnit.INCH);

        if (gamepad2.y) {
           target = -450;
           powerRange = 0.5;
           samplePick = false;
        } else if (gamepad2.a) {
            target = -700;
            powerRange = 0.15;
            samplePick = true;
        }
        else if (gamepad2.b) {
            target = -650;
            powerRange = 0.25;
            samplePick = false;
        }
        if (samplePick && Math.round(distanceInch) <= 3){
            target = ArmMotor1.getCurrentPosition();
            powerRange = 0.15;
        }

        controller.setPID(p, i, d);

        int armPos = ArmMotor1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;


        ArmMotor1.setPower(Range.clip(power * .75,-powerRange,powerRange));

        telemetry.addData("pos", armPos);
        telemetry.addData("pid", pid);
        telemetry.addData("ff", ff);
        telemetry.addData("power ", power);
        telemetry.addData("target ", target);
        telemetry.addData("distance ", distanceInch );
        telemetry.update();



    }
}
