package controlsandbox.lipm;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCylinder3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.euclid.tools.EuclidCoreTools.square;

public class LIPMDynamicsController implements Controller, SCS2YoGraphicHolder
{
   private static double z0 = 1.0;
   private static double mass = 100.0;
   private static double g = 9.81;
   private static double w = Math.sqrt(g / z0);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final double dt;
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;

   private final YoFramePoint3D comPosition;
   private final YoFrameVector3D comVelocity;
   private final YoFrameVector3D comAcceleration;

   private final YoFramePoint3D icpPosition;

   private final YoFramePoint3D basePosition;
   private final YoFramePoint3D comBaseMidpoint;
   private final YoFrameVector3D baseToCom;
   private final YoDouble baseToComLength;

   public LIPMDynamicsController(ControllerInput controllerInput,
                                 ControllerOutput controllerOutput,
                                 double dt)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;
      this.dt = dt;

      comPosition = new YoFramePoint3D("comPosition", ReferenceFrame.getWorldFrame(), registry);
      comVelocity = new YoFrameVector3D("comVelocity", ReferenceFrame.getWorldFrame(), registry);
      comAcceleration = new YoFrameVector3D("comAcceleration", ReferenceFrame.getWorldFrame(), registry);

      icpPosition = new YoFramePoint3D("icpPosition", ReferenceFrame.getWorldFrame(), registry);

      basePosition = new YoFramePoint3D("basePosition", ReferenceFrame.getWorldFrame(), registry);
      comBaseMidpoint = new YoFramePoint3D("comBaseMidpoint", ReferenceFrame.getWorldFrame(), registry);
      baseToCom = new YoFrameVector3D("baseToCom", ReferenceFrame.getWorldFrame(), registry);
      baseToComLength = new YoDouble("baseToComLength", registry);

      comPosition.set(0.0, 0.0, z0);
      basePosition.set(0.0, 0.0, 0.0);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void doControl()
   {
      comBaseMidpoint.interpolate(comPosition, basePosition, 0.5);
      baseToCom.sub(comPosition, basePosition);
      baseToComLength.set(baseToCom.norm());

      comAcceleration.set(baseToCom);
      comAcceleration.setZ(0.0);
      comAcceleration.scale(square(w));

      if (baseToCoM2D() < 0.6)
      {
         comVelocity.scaleAdd(dt, comAcceleration, comVelocity);
         comPosition.scaleAdd(dt, comVelocity, comPosition);
         comPosition.setZ(z0);
      }

      icpPosition.scaleAdd(1.0 / w, comVelocity, comPosition);
      icpPosition.setZ(0.0);

      if (baseToCoM2D() > 0.3)
      {
         basePosition.set(icpPosition);
      }
   }

   private double baseToCoM2D()
   {
      return EuclidCoreTools.norm(baseToCom.getX(), baseToCom.getY());
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("comPositionGraphic",
                                                                    comPosition,
                                                                    0.03,
                                                                    ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("icpPositionGraphic",
                                                                    icpPosition,
                                                                    0.03,
                                                                    ColorDefinitions.Yellow()));

      YoGraphicCylinder3DDefinition baseCylinder = new YoGraphicCylinder3DDefinition();
      baseCylinder.setName("baseCylinder");
      baseCylinder.setColor(ColorDefinitions.Black());
      baseCylinder.setAxis(YoGraphicDefinitionFactory.newYoTuple3DDefinition(baseToCom));
      baseCylinder.setCenter(YoGraphicDefinitionFactory.newYoTuple3DDefinition(comBaseMidpoint));
      baseCylinder.setLength(YoGraphicDefinitionFactory.toPropertyName(baseToComLength));
      baseCylinder.setRadius(0.01);
      group.addChild(baseCylinder);

      return group;
   }

   public static ControllerDefinition createControllerDefinition(double dt)
   {
      return (input, output) -> new LIPMDynamicsController(input, output, dt);
   }
}
