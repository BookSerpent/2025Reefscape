package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class PivotIntakeToAngle extends Command {

    private final IntakeSubsystem intakeSubsystem;

    private final PIDController controller = new PIDController(
        Constants.INTAKE_PIVOT_DOWN_P,
        Constants.INTAKE_PIVOT_I,
        Constants.INTAKE_PIVOT_D
    );

    double targetAngle;
    int debounce;

    public PivotIntakeToAngle(IntakeSubsystem intakeSubsystem, IntakePosition intakePosition) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
        this.targetAngle = intakePosition.getPivotAngleRotations();
    }

    @Override
    public void initialize() {
        debounce = 0;
        // if (intakeSubsystem.getPivotAngle() < targetAngle) {
        //     controller.setP(Constants.INTAKE_PIVOT_DOWN_P);
        // } else {
        //     controller.setP(Constants.INTAKE_PIVOT_UP_P);
        //If going to dealgaenating, try not to slam into robot too hard
        // if (Double.compare(targetAngle, IntakePosition.DEALGAENATING.getPivotAngleRotations()) == 0) {
        //     controller.setP(Constants.INTAKE_GOING_UP_TO_DEALGEANATE);
        // }
        // }

        controller.setP(Constants.INTAKE_PIVOT_DOWN_P);

        controller.setTolerance(0.008);
        controller.setSetpoint(targetAngle);

        System.out.println("START GOING TO " + targetAngle + " with p " + controller.getP());
    }

    @Override
    public void execute() {
        double calcValue = controller.calculate(intakeSubsystem.getPivotAngle());
        System.out.println("Motor spinning at " + intakeSubsystem.getMotorSpeed());
        System.out.println("going to " + controller.getSetpoint() + " at speed of " + calcValue);
        intakeSubsystem.movePivot(calcValue);
        if (controller.atSetpoint()) {
            debounce++;
        } else {
            debounce = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return debounce >= 25;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopPivot();
    }
}
