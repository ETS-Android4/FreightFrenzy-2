package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExampleDriveTrain {

    DcMotor LF;
    DcMotor RF;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public ExampleDriveTrain( HardwareMap hardwareMap, boolean useEncoders ) {

        LF = hardwareMap.dcMotor.get("LF");
        LF.setPower(0);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setDirection(DcMotor.Direction.REVERSE);

        RF = hardwareMap.dcMotor.get("RF");
        RF.setPower(0);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if( useEncoders ) {
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void drive( double x, double y ) {

        if( y > 0 || y < 0 ) {
            LF.setPower(0.75 * y);
            RF.setPower(0.75 * y);

        }
        else if( x > 0 ) {
            LF.setPower(0.75 * x);
            RF.setPower(0.75 * -x);

        }
        else if( x < 0 ) {
            LF.setPower(0.75 * x);
            RF.setPower(0.75 * -x);
        }
        else{
            LF.setPower(0);
            RF.setPower(0);
        }
    }

    public void startMoving( double speed, double leftInches, double rightInches ) {

        // Determine new target position, and pass to motor controller
        int newLeftTarget = LF.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        LF.setTargetPosition( newLeftTarget );

        int newRightTarget = RF.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        RF.setTargetPosition( newRightTarget );

        // Turn On RUN_TO_POSITION
        LF.setMode( DcMotor.RunMode.RUN_TO_POSITION );
        RF.setMode( DcMotor.RunMode.RUN_TO_POSITION );

        LF.setPower( Math.abs(speed) );
        RF.setPower( Math.abs(speed) );
    }

    public void stopMoving() {
        // Stop all motion;
        LF.setPower( 0 );
        RF.setPower( 0 );

        // Turn off RUN_TO_POSITION
        LF.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        RF.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
    }

    public boolean hasReachedTarget() {

        return !( LF.isBusy() && RF.isBusy() );
    }

    public void displayPosition( Telemetry telemetry, String status ) {

        telemetry.addData("Target", "Running to %7d : %7d", LF.getTargetPosition(), RF.getTargetPosition() );
        telemetry.addData("Position", "Running at %7d : %7d", LF.getCurrentPosition(), RF.getCurrentPosition() );
        telemetry.addData("Status", status );
        telemetry.update();
    }
}
