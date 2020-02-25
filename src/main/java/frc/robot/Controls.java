package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIconstants;
import frc.robot.util.Procedure;
import frc.robot.commands.ballmovement.RunIntakeIndex;
import static frc.robot.RobotContainer.*;

public enum Controls {
    AUTO(() -> {
      return new Control();
    }),

    DRIVE(() -> {
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      // Left Y Axis needs to be inverted for driving forward
      Command command = new RunCommand(
          () -> m_drive.arcadeDrive(-1 * m_operatorController.getRawAxis(OIconstants.leftYAxis),
          m_operatorController.getRawAxis(OIconstants.rightXAxis)),
              m_drive);
      return new Control(command);


    }), INTAKEINDEX(() -> {
      return new Control().whenHeld(new JoystickButton(m_testController, Button.kBumperLeft.value),
          new RunIntakeIndex(m_indexer, m_intake));


    }), REVERSEINDEXER(() -> {
      return new Control()
          .whenPressed(new JoystickButton(m_testController, Button.kBumperRight.value), () -> m_indexer.reverse())
          .whenReleased(new JoystickButton(m_testController, Button.kBumperRight.value), () -> m_indexer.disable());
    });

    private static class Control {
      List<Procedure> onInit;
      List<Procedure> whenRunning;
      List<Procedure> onEnd;

      public Control() {
        onInit = new ArrayList<>();
        whenRunning = new ArrayList<>();
        onEnd = new ArrayList<>();
      }

      public Control(Procedure onInit0, Procedure whenRunning0, Procedure onEnd0) {
        this();
        onInit.add(onInit0);
        whenRunning.add(whenRunning0);
        onEnd.add(onEnd0);
      }

      public Control(Command command) {
        onInit.add(() -> command.schedule());
        onEnd.add(() -> command.cancel());
      }

      public Control whenHeld(JoystickButton button, Command command) {
        whenRunning.add(() -> {
          if (button.get()) {
            if (!command.isScheduled())
              command.schedule();
          } else if (command.isScheduled())
            command.cancel();
        });
        onEnd.add(() -> {
          if (command.isScheduled())
            command.cancel();
        });
        return this;
      }

      // TODO add another whenHeld with a procedure
     
      public Control whenPressed(JoystickButton button, Command command) {
        whenRunning.add(() -> {
          if (button.get() && !command.isScheduled())
            command.schedule();
        });
        onEnd.add(() -> {
          if (command.isScheduled())
            command.cancel();
        });
        return this;
      }

      public Control whenPressed(JoystickButton button, Procedure procedure) {
        boolean pressed = false;
        whenRunning.add(() -> {
          if (button.get()) {
            if (!pressed) {
              procedure.invoke();
              pressed = true;
            }
          } else {
            pressed = false;
          }
        });
        return this;
      }

      public Control whenReleased(JoystickButton button, Command command) {
        whenRunning.add(() -> {
          if (!button.get() && !command.isScheduled())
            command.schedule();
        });
        onEnd.add(() -> {
          if (command.isScheduled())
            command.cancel();
        });
        return this;
      }

      public Control whenReleased(JoystickButton button, Procedure procedure) {
        boolean pressed = true;
        button.
        whenRunning.add(() -> {
          if (button.get()) {
            pressed = true;
          } else if (pressed) {
              procedure.invoke();
              pressed = false;
            }
        });
        return this;
      }
    }

    List<Procedure> onInit;
    List<Procedure> whenRunning;
    List<Procedure> onEnd;

    private Controls(Supplier<Control> controlSupplier) {
      Control control = controlSupplier.get();
      onInit = control.onInit;
      whenRunning = control.whenRunning;
      onEnd = control.onEnd;
    }

    public void init() {
      onInit.forEach(n -> n.invoke());
    }

    public void run() {
      whenRunning.forEach(n -> n.invoke());
    }

    public void end() {
      onEnd.forEach(n -> n.invoke());
    }
  }