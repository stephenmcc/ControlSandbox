package controlsandbox;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class CartPoleRobot extends Robot
{
   public static double CART_MASS = 5.0;
   public static double POLE_MASS = 3.0;
   public static double POLE_COM_OFFSET = 0.3;

   private final SliderJoint sliderJoint;
   private final PinJoint pinJoint;

   public CartPoleRobot()
   {
      super("CartPole");

      sliderJoint = new SliderJoint("slider", new Vector3D(), this, Axis3D.X);
      pinJoint = new PinJoint("pin", new Vector3D(), this, Axis3D.Y);
      addRootJoint(sliderJoint);

      Link cartLink = new Link("cartLink");
      cartLink.setMass(CART_MASS);
      cartLink.setMomentOfInertia(0.5, 0.5, 0.5);
      cartLink.setComOffset(new Vector3D());

      Link poleLink = new Link("poleLink");
      poleLink.setMass(POLE_MASS);
      poleLink.setMomentOfInertia(0.2, 1.0, 0.2);
      poleLink.setComOffset(new Vector3D(0.0, 0.0, POLE_COM_OFFSET));

      Graphics3DObject cartGraphics = new Graphics3DObject();
      cartGraphics.addCube(0.1, 0.1, 0.1, true, YoAppearance.Black());
      cartLink.setLinkGraphics(cartGraphics);

      Graphics3DObject poleGraphics = new Graphics3DObject();
      poleGraphics.addCylinder(POLE_COM_OFFSET * 2, 0.01, YoAppearance.Red());
      poleLink.setLinkGraphics(poleGraphics);

      sliderJoint.setLink(cartLink);
      sliderJoint.addJoint(pinJoint);
      pinJoint.setLink(poleLink);
   }

   public SliderJoint getSliderJoint()
   {
      return sliderJoint;
   }

   public PinJoint getPinJoint()
   {
      return pinJoint;
   }
}
