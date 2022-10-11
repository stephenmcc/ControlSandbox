package controlsandbox;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.simulationconstructionset.*;

public class CartPoleRobot
{
   public static double CART_MASS = 5.0;
   public static double POLE_MASS = 3.0;
   public static double POLE_COM_OFFSET = 0.3;
   public static double g = 9.81;

   public static final String CART_JOINT_NAME = "cartJoint";
   public static final String POLE_JOINT_NAME = "poleJoint";

   private final RobotDescription robotDescription;
   private final SliderJointDescription sliderJoint;
   private final PinJointDescription pinJoint;

   private final RobotFromDescription robot;

   private final DMatrixRMaj A_lin;
   private final DMatrixRMaj B_lin;

   public CartPoleRobot()
   {
      robotDescription = new RobotDescription("CartPole");
      sliderJoint = new SliderJointDescription(CART_JOINT_NAME, new Vector3D(), Axis3D.X);
      pinJoint = new PinJointDescription(POLE_JOINT_NAME, new Vector3D(), Axis3D.Y.negated());
      robotDescription.addRootJoint(sliderJoint);

      LinkDescription cartLink = new LinkDescription("cartLink");
      cartLink.setMass(CART_MASS);
      cartLink.setMomentOfInertia(0.5, 0.5, 0.5);
      cartLink.getCenterOfMassOffset(new Vector3D());

      LinkDescription poleLink = new LinkDescription("poleLink");
      poleLink.setMass(POLE_MASS);
      poleLink.setMomentOfInertia(0.2, 1.0, 0.2);
      poleLink.setCenterOfMassOffset(new Vector3D(0.0, 0.0, -POLE_COM_OFFSET));

      LinkGraphicsDescription cartGraphics = new LinkGraphicsDescription();
      cartGraphics.addCube(0.1, 0.1, 0.1, true, YoAppearance.Black());
      cartLink.setLinkGraphics(cartGraphics);

      LinkGraphicsDescription poleGraphics = new LinkGraphicsDescription();
      poleGraphics.translate(0.0, 0.0, -2.0 * POLE_COM_OFFSET);
      poleGraphics.addCylinder(POLE_COM_OFFSET * 2, 0.01, YoAppearance.Red());
      poleLink.setLinkGraphics(poleGraphics);

      sliderJoint.setLink(cartLink);
      sliderJoint.addJoint(pinJoint);
      pinJoint.setLink(poleLink);

      robot = new RobotFromDescription(robotDescription);

      // from https://ocw.mit.edu/courses/6-832-underactuated-robotics-spring-2009/72bc06c4dc73315bf49c28a81dc2b996_MIT6_832s09_read_ch03.pdf
      A_lin = new DMatrixRMaj(4, 4);
      B_lin = new DMatrixRMaj(4, 1);

      A_lin.set(0, 2, 1.0);
      A_lin.set(1, 3, 1.0);
      A_lin.set(2, 1, POLE_MASS * g / CART_MASS);
      A_lin.set(3, 1, (POLE_MASS + CART_MASS) * g / (CART_MASS * POLE_COM_OFFSET));

      B_lin.set(2, 0, 1.0 / CART_MASS);
      B_lin.set(3, 0, 1.0 / (CART_MASS * POLE_COM_OFFSET));
   }

   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   public RobotFromDescription getRobot()
   {
      return robot;
   }

   public OneDegreeOfFreedomJoint getSCSCartJoint()
   {
      return (OneDegreeOfFreedomJoint) robot.getJoint(CART_JOINT_NAME);
   }

   public OneDegreeOfFreedomJoint getSCSPoleJoint()
   {
      return (OneDegreeOfFreedomJoint) robot.getJoint(POLE_JOINT_NAME);
   }

   public DMatrixRMaj getA_lin()
   {
      return A_lin;
   }

   public DMatrixRMaj getB_lin()
   {
      return B_lin;
   }
}
