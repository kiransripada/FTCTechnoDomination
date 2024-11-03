//Leilanie

package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpPT;

public class ArmMotor {
    private RobotParametersPT params;
    public DcMotorEx ArmMotor1;
    public DcMotorEx ArmMotor2;
    PIDFCoefficients pidfOrig = new PIDFCoefficients();
    PIDFCoefficients pidfModified = new PIDFCoefficients();

    int armPos;
    double pid;
    double ff;
    double power;

    private PIDController controller;
    public static double p =0.0180, i=0, d=0.0009;
    public static double f=0.77;

    public static int targetPos = 0;
    private static double ticks_in_degree = 1425.1/180.0;
    boolean targetReached = false;


    public ArmMotor(RobotParametersPT params, HardwareMap hardwareMap) {
        controller = new PIDController(p,i,d);
        ArmMotor1 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.armMotorName1);
        //ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        ArmMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*ArmMotor1 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.armMotorName1);
        ArmMotor2 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.armMotorName2);

        ArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */


    }
        public void stateUpdate (RobotParametersPT.ArmState armState,double power){
            switch (armState) {
                case PIVOT_UP:
                    pivotUp(power);
                    break;

                case PIVOT_DOWN:
                    pivotDown(power);
                    break;

                case STOP:
                    stop();
                    break;
            }
        }

        public void pivotUp (double power){
            ArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmMotor1.setPower(power);

            ArmMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmMotor2.setPower(power);
        }

        public void pivotDown (double power){
            ArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmMotor1.setPower(-power);

            ArmMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmMotor2.setPower(-power);
        }

        public void stop () {
            ArmMotor1.setPower(0);
            ArmMotor2.setPower(0);
        }

    public int getNewPosition(double distance) {
        double Counts_Per_Motor_Arm = RobotParametersPT.Counts_Per_Motor_Arm;
        double Drive_Gear_Reduction = RobotParametersPT.Drive_Gear_Reduction;
        double Arm_Diameter = RobotParametersPT.Arm_Diameter;
        //double Counts_Per_Inch_Arm = (Counts_Per_Motor_Arm * Drive_Gear_Reduction)/(Arm_Diameter * 3.1415);
        return (int)(distance);
    }

    public void moveArm(double distance) {

//        pidfOrig = ArmMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(p, i, d, f);

        ArmMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // Re-read coefficients and verify change.
        pidfModified = ArmMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        int newTarget1 =  (int) getNewPosition(distance);

        ArmMotor1.setTargetPosition((int)distance);
        ArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor1.setPower(0.9);

    }

    public void moveArmNotUsed(double distance) {
        ArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newTarget1 = ArmMotor1.getCurrentPosition() + (int) getNewPosition(distance);
        int newTarget2 = ArmMotor2.getCurrentPosition() + (int) getNewPosition(distance);

        ArmMotor1.setTargetPosition((int)distance);
        ArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor1.setPower(1);

        ArmMotor2.setTargetPosition((int)distance);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor2.setPower(1);

 /*       if (ArmMotor1.getCurrentPosition() > getNewPosition(distance)) {

            ArmMotor1.setPower(0);
            ArmMotor2.setPower(0);
        }
*/

    }
    public int getCurrentPosition(DcMotor ArmMotor) {
        return ArmMotor.getCurrentPosition();
    }

    public String getTelemetry(){
        String telemetry = "";
        telemetry = telemetry + "Arm1 position - " + getCurrentPosition(ArmMotor1) + " ";
        telemetry = telemetry + "P,I,D,F (orig)"+ "%.04f, %.04f, %.04f, %.04f";
        telemetry = telemetry + pidfOrig.p + " " + pidfOrig.i + " " + pidfOrig.d + " " + pidfOrig.f + " ";

        telemetry = telemetry + "P,I,D,F (modified)"+ "%.04f, %.04f, %.04f, %.04f";
        telemetry = telemetry + pidfModified.p + " " + pidfModified.i + " " + pidfModified.d + " " + pidfModified.f + " ";

        return telemetry;
    }

    public void holdArm(double power) {
        ArmMotor1.setPower(power);
        ArmMotor2.setPower(power);

    }
    public void noEncoderMovement(double power){
        ArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor1.setPower(-power);
        ArmMotor2.setPower(power);


    }
    public void moveArmVersion2(int target) {
        p =0.0180;
        i=0;
        d=0.0009;
        f=0.77;
        ticks_in_degree =  1425.1/180.0;



        controller.setPID(p, i, d);
        armPos = ArmMotor1.getCurrentPosition();
        targetPos = target;
        pid = controller.calculate(armPos, targetPos);
        ff = Math.cos(Math.toRadians(targetPos / ticks_in_degree)) * f;

        power = pid + ff;



        ArmMotor1.setPower(power * .75);

    }
    public String getTelemetryForArm(){

        String telemetry = "";
        telemetry = telemetry + "pos - " + ArmMotor1.getCurrentPosition();
        telemetry = telemetry + "pid"+ pid;
        telemetry = telemetry + "ff"+ ff;
        telemetry = telemetry + "power"+ power;
        telemetry = telemetry + "target"+ targetPos;

        return telemetry;
    }
}
//PIDCoefficients pidOrig = motorExLeft.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // change coefficients using methods included with DcMotorEx class.
//        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
//        motorExLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);