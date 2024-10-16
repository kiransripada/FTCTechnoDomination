package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name="WheelClawTest", group="TestOpModes")
public class TestOpModeWheelClaw extends OpMode {

    private Servo ClawCRServo1;
    private Servo ClawCRServo2;

    private RobotParametersPT params;
    private Robot myRobot;

    @Override
    public void init() {
        params = new RobotParametersPT();
        myRobot = new Robot(params,hardwareMap,false,false,true, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            myRobot.clawTurnIn();
        } else if (gamepad2.left_bumper) {
            myRobot.clawTurnOut();
        } else {
            myRobot.clawStop();
        }
    }
}
