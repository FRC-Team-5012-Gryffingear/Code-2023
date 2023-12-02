package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.otherInfo.controllerConstant;
// 2 Talon SRX motors, when pressing A motors shoot outward
public class ArcadeDriveSSTest extends SubsystemBase {
    TalonSRX MainRightM = new TalonSRX(Constants.MainRightMotor);
    TalonSRX MainLeftM = new TalonSRX(Constants.MainLeftMotor);
    TalonSRX BackRightM = new TalonSRX(Constants.BackRightMotor);
    TalonSRX BackLeftM = new TalonSRX(Constants.BackLeftMotor);

    TalonSRX AShoot = new TalonSRX(Constants.ShootA);
    TalonSRX BShoot = new TalonSRX(Constants.ShootB);

  public ArcadeDriveSSTest() {
    MainRightM.configFactoryDefault();
    MainLeftM.configFactoryDefault();
    BackRightM.configFactoryDefault();
    BackLeftM.configFactoryDefault();

    AShoot.configFactoryDefault();
    BShoot.configFactoryDefault();

    MainRightM.setNeutralMode(NeutralMode.Coast);
    MainLeftM.setNeutralMode(NeutralMode.Coast);
    BackRightM.setNeutralMode(NeutralMode.Coast);
    BackLeftM.setNeutralMode(NeutralMode.Coast);

    AShoot.setNeutralMode(NeutralMode.Brake);
    BShoot.setNeutralMode(NeutralMode.Brake);

    BackRightM.follow(MainRightM);
    BackLeftM.follow(MainLeftM);

    BShoot.follow(AShoot);

    MainRightM.setInverted(InvertType.InvertMotorOutput);
    BackRightM.setInverted(InvertType.FollowMaster);

    AShoot.setInverted(InvertType.InvertMotorOutput);
    BShoot.setInverted(InvertType.FollowMaster);

  }

  public void moveAndTurn(double Power, double Turn) {
        MainRightM.set(ControlMode.PercentOutput, Power + Turn);
        MainLeftM.set(ControlMode.PercentOutput, Power - Turn);
  }

  public void turnAndShoot(Boolean P) {
    if(P)
        AShoot.set(ControlMode.PercentOutput, 1);
    else{
        AShoot.set(ControlMode.PercentOutput, 0);
    }
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

