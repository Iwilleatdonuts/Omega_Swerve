package org.firstinspires.ftc.teamcode.Subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Utilities.SwerveModuleConstants
import kotlin.math.abs

class SwerveModule(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry,
    moduleConstants: SwerveModuleConstants
) : SubsystemBase() {
    private val modNumber: Int
    private val drive: DcMotorEx
    private val angle: CRServo
    private val moduleHeading: AnalogInput
    private val controller: PIDFController
    private val moduleOffset: Double
    public var moduleSetpoint: Double

    //Idk if this is how ur supposed to make a swervy drive but I'm gonna
    // put a boolean to tell the module when it is backwards and so the
    // drivy motor should be backwards because im rly smart definetely yes yes i can speel
    private var isModuleBackwards = false

    init {
        drive = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, moduleConstants.driveMotor)
        angle = hardwareMap.get<CRServo>(CRServo::class.java, moduleConstants.angleServo)

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        drive.setDirection(DcMotorSimple.Direction.FORWARD)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        drive.setVelocityPIDFCoefficients(1.0, 0.0, 0.0, 0.00036)

        angle.setDirection(DcMotorSimple.Direction.FORWARD)

        controller = PIDFController(
            moduleConstants.kP,
            moduleConstants.kI,
            moduleConstants.kD,
            moduleConstants.kF
        )


        moduleHeading =
            hardwareMap.get<AnalogInput>(AnalogInput::class.java, moduleConstants.feedback)
        moduleOffset = moduleConstants.moduleOffset

        moduleSetpoint = getDegrees(true)

        modNumber = moduleConstants.modNumber
    }

    fun setDrivePower(power: Double) {
        var newPower = power

        if (isModuleBackwards) {
            newPower = -newPower
        }
        //TODO figure out velocity stuff, change PID coefficients to run veloicty
//        drive.setVelocity(newPower*6000);
        drive.setPower(newPower)
    }

    //take a value from -1 to 1
    fun setTurnSpeed(speed: Double) {
        angle.setPower(speed * 0.5)
    }

    val rawAngle: Double
        get() = moduleHeading.getVoltage()

    //withOffset set to true will return real angle, else will return raw angle
    fun getDegrees(withOffset: Boolean): Double {
        val rawAngle = (this.rawAngle / moduleHeading.getMaxVoltage()) * 360

        var realAngle = rawAngle - moduleOffset

        realAngle = (realAngle + 360) % 360

        val realRealAngle = (360 - realAngle) % 360

        return if (withOffset) realRealAngle else rawAngle
    }

    fun getWrappedError(setpoint: Double, measurement: Double): Double {
        var error = setpoint - measurement
        error = ((error + 180) % 360 + 360) % 360 - 180
        return error
    }

    fun getModuleSetpoint(): Double {
        return moduleSetpoint
    }

    fun setModuleSetpoint(setpoint: Double) {
        var newSetpoint = setpoint

        val error = abs(getWrappedError(newSetpoint, getDegrees(true)))

        if (error > 90) {
            newSetpoint = (newSetpoint + 180) % 360
            isModuleBackwards = true
        } else {
            isModuleBackwards = false
        }

        moduleSetpoint = newSetpoint
    }

    fun setModulePosition() {
        val error = getWrappedError(moduleSetpoint, getDegrees(true))

        val placeholder = moduleSetpoint - error

        setTurnSpeed(controller.calculate(placeholder, moduleSetpoint))
    }

    fun atRoughSepoint(): Boolean {
        val error = getWrappedError(getModuleSetpoint(), getDegrees(true))

        return error < 10
    }

    fun update() {
        telemetry.addLine("Module " + modNumber)
        telemetry.addData("Degrees \t", getDegrees(true))
        telemetry.addData("Raw Angle \t", getDegrees(false))
        telemetry.addData("Setpoint \t", getModuleSetpoint())
        telemetry.addData("Drive speed \t", drive.getVelocity())
        telemetry.addLine()
    }
}
