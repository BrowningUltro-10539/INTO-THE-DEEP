package org.firstinspires.ftc.teamcode.common.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


@Autonomous
public class TimeBasedRight extends LinearOpMode {
    Robot robot = new Robot(hardwareMap, false);// Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();



    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");




    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and .

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

}
