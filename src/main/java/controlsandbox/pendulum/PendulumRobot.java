package controlsandbox.pendulum;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.RobotFromDescription;

public class PendulumRobot
{
   public static double MASS = 5.0;
   public static double INERTIA = 0.16;
   public static double LENGTH = 0.5;
   public static double COM_OFFSET = 0.22;
   public static double g = 9.81;

   public static final String JOINT_NAME = "pendulum";

   private final RobotDescription robotDescription;
   private final PinJointDescription joint;
   private final RobotFromDescription robot;

   public PendulumRobot()
   {
      robotDescription = new RobotDescription("Pendulum");
      joint = new PinJointDescription(JOINT_NAME, new Vector3D(), Axis3D.X);
      robotDescription.addRootJoint(joint);

      LinkDescription shoulderLink = new LinkDescription("link");
      shoulderLink.setMass(MASS);
      shoulderLink.setMomentOfInertia(INERTIA, INERTIA, INERTIA);
      shoulderLink.setCenterOfMassOffset(0.0, 0.0, -COM_OFFSET);

      LinkGraphicsDescription shoulderGraphics = new LinkGraphicsDescription();
      shoulderGraphics.translate(0.0, 0.0, -LENGTH);
      shoulderGraphics.addCylinder(LENGTH, 0.015, YoAppearance.Red());
      shoulderLink.setLinkGraphics(shoulderGraphics);

      robot = new RobotFromDescription(robotDescription);
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
