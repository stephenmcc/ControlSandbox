package controlsandbox.pendulum;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.simulationconstructionset.RobotFromDescription;

public class PendulumRobot
{
   public static double MASS = 5.0;
   public static double INERTIA = 0.16;
   public static double LENGTH = 0.5;
   public static double COM_OFFSET = 0.22;
   public static double g = 9.81;

   public static final String JOINT_NAME = "pendulum";

   private final RobotDefinition robotDefinition;
   private final RigidBodyDefinition rootBodyDefinition;
   private final RevoluteJointDefinition joint;
   private final RobotFromDescription robot;

   public PendulumRobot()
   {
      robotDefinition = new RobotDefinition("Pendulum");
      rootBodyDefinition = new RigidBodyDefinition("root");
      joint = new RevoluteJointDefinition(JOINT_NAME, new Vector3D(), Axis3D.X);
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);
      rootBodyDefinition.addChildJoint(joint);

      RigidBodyDefinition link = new RigidBodyDefinition("link");
      link.setMass(MASS);
      link.setMomentOfInertia(new MomentOfInertiaDefinition(INERTIA, INERTIA, INERTIA));
      link.setCenterOfMassOffset(0.0, 0.0, -COM_OFFSET);

      VisualDefinitionFactory graphics = new VisualDefinitionFactory();
      graphics.appendTranslation(0.0, 0.0, -0.5 * LENGTH);
      graphics.addCylinder(LENGTH, 0.015, ColorDefinitions.Blue());
      link.addVisualDefinitions(graphics.getVisualDefinitions());

      robot = new RobotFromDescription(robotDefinition);
   }

   public RobotFromDescription getRobot()
   {
      return robot;
   }

   public double getMaxTorque()
   {
      return 0.5 * g * COM_OFFSET;
   }
}
