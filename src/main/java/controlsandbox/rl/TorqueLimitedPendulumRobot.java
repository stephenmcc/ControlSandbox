package controlsandbox.rl;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

public class TorqueLimitedPendulumRobot extends RobotDefinition
{
   public TorqueLimitedPendulumRobot()
   {
      super("ToqueLimitedPendulumRobot");
      RigidBodyDefinition root = new RigidBodyDefinition("root");
      RevoluteJointDefinition joint = new RevoluteJointDefinition("joint", new Point3D(), Axis3D.X);

      RigidBodyDefinition link = new RigidBodyDefinition("link");
      link.setMass(TorqueLimitedPendulum.MASS);
      link.setMomentOfInertia(new MomentOfInertiaDefinition(TorqueLimitedPendulum.MASS, TorqueLimitedPendulum.MASS, TorqueLimitedPendulum.MASS));
      link.setCenterOfMassOffset(0.0, 0.0, -TorqueLimitedPendulum.LENGTH);

      VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
      visualDefinitionFactory.appendTranslation(0.0, 0.0, -0.5 * TorqueLimitedPendulum.LENGTH);
      visualDefinitionFactory.addCylinder(TorqueLimitedPendulum.LENGTH, 0.02, ColorDefinitions.Yellow());
      joint.getSuccessor().getVisualDefinitions().addAll(visualDefinitionFactory.getVisualDefinitions());

      root.addChildJoint(joint);
      joint.setSuccessor(link);
      setRootBodyDefinition(root);

//      addControllerDefinition();
   }

   private class PendulumController implements Controller
   {
      @Override
      public void doControl()
      {

      }
   }
}
