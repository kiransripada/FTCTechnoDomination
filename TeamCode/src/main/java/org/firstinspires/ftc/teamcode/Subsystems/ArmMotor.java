//Leilanie

package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

public class ArmMotor {
    private RobotParametersPT params;
    public DcMotor ArmMotor;



    public ArmMotor(RobotParametersPT params, HardwareMap hardwareMap) {
        ArmMotor = hardwareMap.get(DcMotor.class, params.armMotorName);

        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmMotor.setPower(power);
        }

        public void pivotDown (double power){
            ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmMotor.setPower(-power);
        }

        public void stop () {
            ArmMotor.setPower(0);
        }

    public int getNewPosition(double distance) {
        double Counts_Per_Motor_Arm = params.Counts_Per_Motor_Arm;
        double Drive_Gear_Reduction = params.Drive_Gear_Reduction;
        double Arm_Diameter = params.Arm_Diameter;
       // double Counts_Per_Inch_Arm = (Counts_Per_Motor_Arm * Drive_Gear_Reduction)/(Arm_Diameter * 3.1415);
        return (int)(distance);
    }
    public void moveArm(double distance) {
        int newTarget = ArmMotor.getCurrentPosition() + (int) getNewPosition(distance);

        ArmMotor.setTargetPosition((int)distance);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(-.75);

    }
    public int getCurrentPosition() {
        return ArmMotor.getCurrentPosition();
    }

}