package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase
{
    // Order of setpoint encoder values: L1(placeholder), L2, L3, L4, Net scoring
    private double[] setPoints = {0, 10, 25.28395062, 25.28395062, 21.33333333, 20.54320988, 30, 34, 15, 1.7, 17}; // 0 is placeholder for L1 (NOT YET DESIGNED)
    //margin of error for movement
    private double errorRange = 0.25;
    
    //identifies claw properties
    private TalonFX clawRotate;
    private TalonFX clawSpin;
    private CANrange clawTOF;
    private TalonFX clawFunnel;
    private InvertedValue CounterClockwise_Positive = InvertedValue.Clockwise_Positive;
    
    // line below potentially incorrect; poition voltage construtor parameter 
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public Claw()
    {
        //initates the properities with IDs
        clawRotate = new TalonFX(Constants.ClawConstants.clawID1);
        clawSpin = new TalonFX(Constants.ClawConstants.clawID2);
        clawTOF = new CANrange(Constants.ClawConstants.canRangeID);
        clawFunnel = new TalonFX(Constants.ClawConstants.clawID3);
        
        //Gives the motors configuration values to be modified
        clawRotate.getConfigurator().apply(new TalonFXConfiguration());
        clawSpin.getConfigurator().apply(new TalonFXConfiguration());
        clawFunnel.getConfigurator().apply(new TalonFXConfiguration());

        //Sets PID values for claw rotation
        TalonFXConfiguration configsRotate = new TalonFXConfiguration();
        configsRotate.Slot0.kP = Constants.ClawConstants.claw_P;
        configsRotate.Slot0.kI = Constants.ClawConstants.claw_I;
        configsRotate.Slot0.kD = Constants.ClawConstants.claw_D;

        configsRotate.Slot1.kP = Constants.ClawConstants.claw_P6000;
        configsRotate.Slot1.kI = Constants.ClawConstants.claw_I;
        configsRotate.Slot1.kD = Constants.ClawConstants.claw_D;

        configsRotate.Slot2.kP = Constants.ClawConstants.claw_P12000;
        configsRotate.Slot2.kI = Constants.ClawConstants.claw_I;
        configsRotate.Slot2.kD = Constants.ClawConstants.claw_D;


        

        configsRotate.withMotorOutput(new MotorOutputConfigs().withInverted(CounterClockwise_Positive));

        configsRotate.CurrentLimits.withStatorCurrentLimit(35);
        configsRotate.CurrentLimits.withStatorCurrentLimitEnable(true);

        TalonFXConfiguration configsSpin = new TalonFXConfiguration();
        configsSpin.CurrentLimits.withStatorCurrentLimit(30);
        configsSpin.CurrentLimits.withStatorCurrentLimitEnable(true);

        clawRotate.getConfigurator().apply(configsRotate);
        clawSpin.getConfigurator().apply(configsSpin);
        clawFunnel.getConfigurator().apply(configsSpin);
    }

    public void spinRollers(double OutputPercent)
    {
        OutputPercent /= 100.;
        clawSpin.set(-OutputPercent);
        clawFunnel.set(OutputPercent);
    }

    public boolean algaeDetected()
    {        
        SmartDashboard.putBoolean("Algae Detect", getTOFDistance() < .2);
        return getTOFDistance() < .06;
    }

    public boolean TOFdetect()
    {
        return clawTOF.getIsDetected().getValue();
    }
    public boolean pieceDetected()
    {
        SmartDashboard.putBoolean("Piece Detect", getTOFDistance() < .4);
        return getTOFDistance() < .1;
    }

    public double getTOFDistance()
    {
        return clawTOF.getDistance().getValueAsDouble();
    }


    public double getSpin()
    {
        return clawSpin.get();
    }
    public double getV()
    {
        return clawRotate.get();
    }

    public void customPosition(double setPoint)
    {
        clawRotate.setControl(m_request.withPosition(setPoint).withSlot(0));
    }

    public void toPosition(int setpointIndex) 
    {
        clawRotate.setControl(m_request.withPosition(setPoints[setpointIndex]).withSlot(0));
    }
    
    public void toPosition6000(int setpointIndex) 
    {
        clawRotate.setControl(m_request.withPosition(setPoints[setpointIndex]).withSlot(1));
    }
    public void toPosition12000(int setpointIndex) 
    {
        clawRotate.setControl(m_request.withPosition(setPoints[setpointIndex]).withSlot(2));
    }

    public double getRollers()
    {
        return clawSpin.get();
    }
    public double getPosition()
    {
        return clawRotate.getPosition().getValueAsDouble();
    }

    public boolean checkSetpoint(int level)
    {
        if (getPosition() < setPoints[level] + errorRange && getPosition() > setPoints[level] - errorRange)
        {
            return true;
        }
        
        return false;
    }

    public void autoIntake()
    {
        if (pieceDetected())
        {
            spinRollers(0);
        }
        else
        {
            spinRollers(50);
        }
    }

    public boolean checkRotation(Double point)
    {
        if (getPosition() < point + errorRange && getPosition() > point - errorRange)
        {
            return true;
        }

        return false;
    }
}