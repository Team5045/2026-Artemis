package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;

public class IntakeCommand extends Command{
    private final IntakeWheels intakeWheels;

    public IntakeCommand(IntakeWheels intakeWheels, CommandXboxController controller)
    {
        this.intakeWheels = intakeWheels;
    }

    public void execute()
    {
        // test
        intakeWheels.setMotorSpeeds(0);
    }
}
