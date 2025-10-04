package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//read claw and elevator first
public class Intake extends SubsystemBase {
    // Order of setpoint encoder values: Full stow, stow with algae, algae intake
    private double[] setPoints = {0, -0.866812, -13.,-7.86899, -8};

    private TalonFX intakeMotor;
    private TalonFX RollerMotor;

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);


    public Intake() 
    {
        
        intakeMotor = new TalonFX(Constants.IntakeConstants.intakeID1);
        RollerMotor = new TalonFX(Constants.IntakeConstants.intakeID2);
        
        //configures the motors
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
        RollerMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        //setups the PID value for the intake
        intakeConfigs.Slot0.kP = Constants.IntakeConstants.intake_P;
        intakeConfigs.Slot0.kI = Constants.IntakeConstants.intake_I;
        intakeConfigs.Slot0.kD = Constants.IntakeConstants.intake_D; 

        //Extra slower movement
        intakeConfigs.Slot1.kD = Constants.IntakeConstants.intake_P6000; 

        
        intakeConfigs.CurrentLimits.withStatorCurrentLimit(25);
        intakeConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        TalonFXConfiguration RollerConfigs = new TalonFXConfiguration();
        RollerConfigs.CurrentLimits.withStatorCurrentLimit(20);
        RollerConfigs.Voltage.withPeakForwardVoltage(8);
        RollerConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(20)
            .withPeakReverseTorqueCurrent(-20);

        intakeMotor.getConfigurator().apply(intakeConfigs);
        RollerMotor.getConfigurator().apply(RollerConfigs);
    }
    
    public void setRollers(double OutputPercent)
    {
      OutputPercent /= 100.;
      RollerMotor.set(-OutputPercent);
    }

    public void toSetpoint(int setpointIndex)
    {
        intakeMotor.setControl(m_request.withPosition(setPoints[setpointIndex]).withSlot(0));
    }
    public void toSetpoint6000(int setpointIndex)
    {
        intakeMotor.setControl(m_request.withPosition(setPoints[setpointIndex]).withSlot(1));
    }
    public void customPosition(double setPoint)
    {
        intakeMotor.setControl(m_request.withPosition(setPoint).withSlot(0));
    }
    public double getV()
    {
        return intakeMotor.get();
    }

    public double getPosition()
    {
        return intakeMotor.getPosition().getValueAsDouble();
    }
}

