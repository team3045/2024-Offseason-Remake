package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        SwerveRequest appliedRequest = requestSupplier.get();

        

        if(appliedRequest instanceof SwerveRequest.FieldCentric){
            double desiredAccelX = (((SwerveRequest.FieldCentric)appliedRequest).VelocityX - getState().speeds.vxMetersPerSecond) / GlobalConstants.tLoop;
            double desiredAccelY = (((SwerveRequest.FieldCentric)appliedRequest).VelocityY - getState().speeds.vyMetersPerSecond) / GlobalConstants.tLoop;
            double desiredAccelTheta = (((SwerveRequest.FieldCentric)appliedRequest).RotationalRate - getState().speeds.omegaRadiansPerSecond) / GlobalConstants.tLoop;

            double requestedAccelX = Math.abs(desiredAccelX) > TunerConstants.kMaxAccelXY  
                ? Math.copySign(TunerConstants.kMaxAccelXY, desiredAccelX)
                : desiredAccelX;
            double requestedAccelY = Math.abs(desiredAccelY) > TunerConstants.kMaxAccelXY
                ? Math.copySign(TunerConstants.kMaxAccelXY, desiredAccelY)
                : desiredAccelY;
            double requestedAccelTheta = Math.abs(desiredAccelTheta) > TunerConstants.kMaxAccelTheta
                ? Math.copySign(TunerConstants.kMaxAccelTheta, desiredAccelTheta)
                : desiredAccelTheta;

            SwerveRequest newReq = ((SwerveRequest.FieldCentric)appliedRequest)
                .withVelocityX(requestedAccelX * GlobalConstants.tLoop + getState().speeds.vxMetersPerSecond)
                .withVelocityY(requestedAccelY * GlobalConstants.tLoop + getState().speeds.vyMetersPerSecond)
                .withRotationalRate(requestedAccelTheta * GlobalConstants.tLoop + getState().speeds.omegaRadiansPerSecond);
            
            return run(() -> this.setControl(newReq));
        }

        return run(() -> this.setControl(appliedRequest));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
