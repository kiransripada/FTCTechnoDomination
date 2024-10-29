//Leilanie

package org.firstinspires.ftc.teamcode.Subsystems;
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


    public ArmMotor(RobotParametersPT params, HardwareMap hardwareMap) {
        ArmMotor1 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.armMotorName1);
        ArmMotor2 = hardwareMap.get(DcMotorEx.class, RobotParametersPT.armMotorName2);

        ArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
        PIDFCoefficients pidfNew = new PIDFCoefficients(TeleOpPT.P, TeleOpPT.I, TeleOpPT.D, TeleOpPT.F);

        ArmMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // Re-read coefficients and verify change.
        pidfModified = ArmMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        int newTarget1 =  (int) getNewPosition(distance);

        ArmMotor1.setTargetPosition((int)distance);
        ArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor1.setPower(0.5);

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





}


//PIDCoefficients pidOrig = motorExLeft.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // change coefficients using methods included with DcMotorEx class.
//        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
//        motorExLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);