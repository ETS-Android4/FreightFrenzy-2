package org.firstinspires.ftc.teamcode.example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ExampleDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.ExampleServo;

@Autonomous(name = "ExampleAutonomous", group="Example")
public class ExampleAutonomous extends LinearOpMode {

    ExampleDriveTrain driveTrain;
    private ElapsedTime runtime = new ElapsedTime();

    static final double CLAW_OPEN = 0.714;
    static final double CLAW_CLOSED = 0.970;

    public void runOpMode() {

        driveTrain = new ExampleDriveTrain( hardwareMap, true );
        ExampleServo claw = new ExampleServo( hardwareMap, "claw");
        ExampleServo arm = new ExampleServo( hardwareMap, "arm");

        waitForStart();

        drive( 0.5, 24.0, 24.0, 30.0 );

        sleep(5000 );
     }

    public void drive( double speed, double leftInches, double rightInches, double timeout ) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset the timeout time
            runtime.reset();

            driveTrain.startMoving( speed, leftInches, rightInches );

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while( opModeIsActive() &&
                    runtime.seconds() < timeout &&
                    !driveTrain.hasReachedTarget() ) {

                // Display it for the driver.
                telemetry.addData("Target","Running to %7d : %7d", driveTrain.getLeftEncoderTarget(), driveTrain.getRightEncoderTarget() );
                telemetry.addData("Position","Running at %7d : %7d", driveTrain.getLeftEncoderPosition(), driveTrain.getRightEncoderPosition() );
                telemetry.addData("Status","Running" );
                telemetry.update();
            }

            driveTrain.stopMoving();

            telemetry.addData("Target","Running to %7d : %7d", driveTrain.getLeftEncoderTarget(), driveTrain.getRightEncoderTarget() );
            telemetry.addData("Position","Running at %7d : %7d", driveTrain.getLeftEncoderPosition(), driveTrain.getRightEncoderPosition() );
            telemetry.addData("Status","Stopped" );
            telemetry.update();

            //  sleep(250);   // optional pause after each move
        }
    }
}
