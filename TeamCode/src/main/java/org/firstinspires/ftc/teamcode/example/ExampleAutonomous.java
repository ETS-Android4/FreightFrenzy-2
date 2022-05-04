package org.firstinspires.ftc.teamcode.example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ExampleDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.ExampleServo;
import org.firstinspires.ftc.teamcode.hardware.OpModeIsActive;

@Autonomous(name = "ExampleAutonomous", group="Example")
public class ExampleAutonomous extends LinearOpMode implements OpModeIsActive {

    ExampleDriveTrain driveTrain;

    static final double CLAW_OPEN = 0.714;
    static final double CLAW_CLOSED = 0.970;

    public void runOpMode() {

        driveTrain = new ExampleDriveTrain( hardwareMap, true );
        ExampleServo claw = new ExampleServo( hardwareMap, "claw");
        ExampleServo arm = new ExampleServo( hardwareMap, "arm");

        waitForStart();

        driveTrain.drive( 0.5, 24.0, 24.0, 30.0, this, telemetry );

        sleep(5000 );
     }

    @Override
    public boolean isActive() {
        return opModeIsActive();
    }
}
