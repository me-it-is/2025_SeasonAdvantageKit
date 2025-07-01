package frc.robot.subsystems.manipulator;
import static frc.robot.Constants.ManipulatorConstants.*;
import static edu.wpi.first.units.Units.Rotations;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorIOSparkMax implements ManipulatorIO {
    private final SparkMax pivot;
    private final SparkMax rollers;
    private SparkMaxConfig pivotConfig;
    private SparkMaxConfig rollerConfig;

    private SparkClosedLoopController pivotController;
    private AbsoluteEncoder pivotEncoder;

    private DigitalInput beamBreak;

    public ManipulatorIOSparkMax (SparkMax pivot, SparkMax rollers) {
        this.pivot = pivot;
        this.rollers = rollers;
        this.pivotConfig = new SparkMaxConfig();
        this.rollerConfig = new SparkMaxConfig();
        this.pivotEncoder = pivot.getAbsoluteEncoder();
        this.pivotController = pivot.getClosedLoopController();

        this.beamBreak = new DigitalInput(kBeamBreakPort);

        pivotConfig.inverted(false).smartCurrentLimit(ManipulatorConstants.currentLimit);
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kP, kI, kD);
        rollerConfig.idleMode(IdleMode.kCoast);

        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollers.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs (ManipulatorIOInputs inputs) {
        inputs.manipulatorAngle = Rotations.of(pivotEncoder.getPosition());
        inputs.rollerOutput = rollers.getAppliedOutput();
        inputs.pivotOutput = pivot.getAppliedOutput();
        inputs.beamBroken = beamBreak.get();
    }

    public void setAngle (Angle angle) {
        pivotController.setReference(
        angle.in(Rotations),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
    }

    public void setRollers (double speed) {
        rollers.set(speed);
    }

    public void close () {
        pivot.close();
        rollers.close();
    }
}
