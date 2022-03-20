package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExampleDriveTrain {

    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;

    public ExampleDriveTrain(HardwareMap hardwareMap ) {

        try {
            LF = hardwareMap.dcMotor.get("LF");
            LF.setPower(0);
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LF.setDirection(DcMotor.Direction.REVERSE);

        } catch (Exception a) {
            LF = null;
        }
        try {
            RF = hardwareMap.dcMotor.get("RF");
            RF.setPower(0);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception a) {
            RF = null;
        }

        try {
            LB = hardwareMap.dcMotor.get("LB");
            LB.setPower(0);
            LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LB.setDirection((DcMotor.Direction.REVERSE));
        } catch (Exception a) {
            LB = null;
        }

        try {
            RB = hardwareMap.dcMotor.get("RB");
            RB.setPower(0);
            RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception a) {
            RB = null;
        }
    }

    public void drive( double x, double y ) {

        if( y > 0 || y < 0 ) {
            LF.setPower(0.75 * y);
            RF.setPower(0.75 * y);
            LB.setPower(0.75 * y);
            RB.setPower(0.75 * y);

        }
        else if( x > 0 ) {
            LF.setPower(0.75 * x);
            RF.setPower(0.75 * -x);
            LB.setPower(0.75 * x);
            RB.setPower(0.75 * -x);

        }
        else if( x < 0 ) {
            LF.setPower(0.75 * x);
            RF.setPower(0.75 * -x);
            LB.setPower(0.75 * x);
            RB.setPower(0.75 * -x);
        }
        else{
            LF.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
            RB.setPower(0);
        }
    }
}
