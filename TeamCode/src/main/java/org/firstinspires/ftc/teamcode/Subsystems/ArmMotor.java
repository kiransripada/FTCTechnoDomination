package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

public class ArmMotor {
    private RobotParametersPT params;
    private DcMotor ArmMotor;



    public ArmMotor(RobotParametersPT params, HardwareMap hardwareMap) {
        ArmMotor = hardwareMap.get(DcMotor.class, params.armMotorName);

        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
            ArmMotor.setPower(power);
        }

        public void pivotDown (double power){
            ArmMotor.setPower(-power);
        }

        public void stop () {
            ArmMotor.setPower(0);
        }

    public int getNewPosition(double distance) {
        double Counts_Per_Motor_Rev = params.Counts_Per_Motor_Rev;
        double Drive_Gear_Reduction = params.Drive_Gear_Reduction;
        double Wheel_Diameter = params.Wheel_Diameter;
        double Counts_Per_Inch = (Counts_Per_Motor_Rev * Drive_Gear_Reduction)/(Wheel_Diameter * 3.1415);
        return (int)(distance * Counts_Per_Inch);
    }
    public void moveArm() {
        int newTarget = ArmMotor.getCurrentPosition() + (int)(getNewPosition(100));

        ArmMotor.setTargetPosition(newTarget);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(0.5);
        ArmMotor.setPower(0.5);
    }


}