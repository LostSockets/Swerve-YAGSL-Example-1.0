package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.operator.ShooterSubsystem;

public class ShooterCmd extends Command{

    private final ShooterSubsystem shooterSubsystem;
    private final double speed;

    public ShooterCmd(ShooterSubsystem shooterSubsystem, double speed){
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      shooterSubsystem.setMotor(speed);
      //System.out.println("speed" + speed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      shooterSubsystem.setMotor(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
  
      return false;
    }
}


