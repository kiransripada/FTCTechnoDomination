package org.firstinspires.ftc.teamcode.Hardware;
//JJ
public final class RobotParametersPT{
    public static final String frontLeftMotorName = "FrontLeftDCMotor";
    public static final String frontRightMotorName = "FrontRightDCMotor";
    public static final String backLeftMotorName = "BackLeftDCMotor";
    public static final String backRightMotorName = "BackRightDCMotor";
    public static final String slideMotorName1 = "SlideMotor1";
    public static final String slideMotorName2 = "SlideMotor2";
    public static final String ClawServoName1 = "ClawServo1";
    public static final String ClawServoName2 = "ClawServo2";
    public static final String armMotorName1 = "ArmMotor1";
    public static final String armMotorName2 = "ArmMotor2";
    public static final double defaultArmPower = 0.75;
    public static final double defaultDrivePower = 0.5;
    public static final double defaultSlidePower = 0.75;
    public static final double defaultClawPower = 0.75;
    public static final double defaultTurnPower = 0.25;

    public static final double powerReduction = 0.8;
    public static final double Counts_Per_Motor_Arm = 537.7;
    public static final double Counts_Per_Motor_Wheel = 384.5;
    public static final double Drive_Gear_Reduction = 1.0;
    public static final double Wheel_Diameter = 3.78;
    public static final double Arm_Diameter = 30;
    public double Counts_Per_Motor_Rev;

    public enum IntakeState {
        PULL_IN,
        PUSH_OUT,
        STOP
    }

    public enum SlideState {
        SLIDE_IN,
        SLIDE_OUT,
        STOP
    }

    public enum ClawState {
        TURN_IN,
        TURN_OUT,
        STOP
    }

    public enum ArmState {
        PIVOT_UP,
        PIVOT_DOWN,
        STOP
    }

}




/*import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotParametersPT {

    public DcMotor FrontLeftDCMotor = null;
    public DcMotor FrontRightDCMotor = null;
    public DcMotor BackLeftDCMotor = null;
    public DcMotor BackRightDCMotor = null;
    public DcMotor IntakeMotor = null;
    public DcMotor SlideMotor = null;

    public void init(HardwareMap hardwareMap) {
        // Initialize drive motors
        FrontLeftDCMotor = hardwareMap.get(DcMotor.class, "FrontLeftDCMotor");
        FrontRightDCMotor = hardwareMap.get(DcMotor.class, "FrontRightDCMotor");
        BackLeftDCMotor = hardwareMap.get(DcMotor.class, "BackLeftDCMotor");
        BackRightDCMotor = hardwareMap.get(DcMotor.class, "BackRightDCMotor");

        // Set motor directions
        FrontLeftDCMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightDCMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeftDCMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightDCMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to brake when power is zero
        FrontLeftDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize intake motor
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        // Initialize slide motor
        SlideMotor = hardwareMap.get(DcMotor.class, "SlideMotor");
        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}*/
