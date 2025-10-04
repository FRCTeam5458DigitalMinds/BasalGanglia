package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    //Order of setpoint encoder values: 0, L1, L2, L3, L4, Net scoring
    //48-3.75 == 44.25
    //48-9.12 = 39
    //setup objects
    private double[] stages = {0, 0, 4.25, 17.5, 41, 0,0};
    private double errorRange = 1.25;
    
    private int elevatorID1 = Constants.ClimbConstants.climbID1;
    private int elevatorID2 = Constants.ClimbConstants.climbID2;

    private SparkMax elevator1;
    private SparkMax elevator2;
    
    private SparkMaxConfig elevatorConfig1;
    private SparkMaxConfig elevatorConfig2;

    private SparkClosedLoopController elevatorController;
    
    public Elevator() {
        
        elevator2 = new SparkMax(elevatorID2, MotorType.kBrushless);
        elevator1 = new SparkMax(elevatorID1, MotorType.kBrushless);

        elevatorConfig1 = new SparkMaxConfig();
        elevatorConfig2 = new SparkMaxConfig();

        elevatorController = elevator1.getClosedLoopController();

        //need not apply PIDF to motor2, as it will follow leader motor1
        elevatorConfig1
            .smartCurrentLimit(60)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            //init-ing pidf values (change thru constants file)
            .closedLoop
                .pidf
                (
                    Constants.ClimbConstants.climb_P, 
                    Constants.ClimbConstants.climb_I, 
                    Constants.ClimbConstants.climb_D, 
                    Constants.ClimbConstants.climb_FF,
                    ClosedLoopSlot.kSlot0
                )
                .pidf
                (
                    Constants.ClimbConstants.climb_P6000, 
                    Constants.ClimbConstants.climb_I, 
                    Constants.ClimbConstants.climb_D, 
                    Constants.ClimbConstants.climb_FF,
                    ClosedLoopSlot.kSlot1
                )
                .outputRange(-1, 1);

        //applying configs motor2, make sure this one is INVERTED (robot will break)
        elevatorConfig2
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake)
            .follow(elevatorID1, true);

        //applying configs to motors
        elevator1.configure(elevatorConfig1,com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator2.configure(elevatorConfig2,com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void changeStage(int stageIndex) 
    {
        //sets up algae pickup
        if (stageIndex > 10)
        {
            if (stageIndex == 12)
            {
                elevatorController.setReference(stages[stageIndex - 10], ControlType.kPosition, ClosedLoopSlot.kSlot0);
            }
            else {
                elevatorController.setReference(stages[stageIndex - 10] + 1, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            }
        }
        //sets up eject coral
        else 
        {
            elevatorController.setReference(stages[stageIndex], ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }
    //same method but different pid values
    public void changeStage6000(int stageIndex) 
    {
        if (stageIndex > 10)
        {
            if (stageIndex == 12)
            {
                elevatorController.setReference(stages[stageIndex - 10], ControlType.kPosition, ClosedLoopSlot.kSlot1);
            }
            else {
                elevatorController.setReference(stages[stageIndex - 10] + 1, ControlType.kPosition, ClosedLoopSlot.kSlot1);
            }
        }
        else 
        {
            elevatorController.setReference(stages[stageIndex], ControlType.kPosition, ClosedLoopSlot.kSlot1);
        }
    }
    //checks if elevator matches stage
    public boolean checkStage(int stage)
    {
        if (getPosition() < stages[stage] + errorRange && getPosition() > stages[stage] - errorRange)
        {
            return true;
        }

        return false;
    }
    //get velocity
    public double getV()
    {
        return elevator1.get();
    }
    //get encoder of value of elevator motor
    public double getPosition()
    {
        return elevator1.getEncoder().getPosition();
    }
    //checks if the elevator position is within the L4 range
    public boolean checkL4()
    {
        if (getPosition() > 20 && getPosition() < 21)
        {
            return true;
        }
        return false;
    }
}