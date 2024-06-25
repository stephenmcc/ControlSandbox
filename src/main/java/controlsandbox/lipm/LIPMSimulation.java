package controlsandbox.lipm;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicBox3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.simulation.robot.controller.RobotControllerManager;

public class LIPMSimulation
{
   public LIPMSimulation()
   {
      PlaceholderRobot lipm = new PlaceholderRobot();

      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2("LIPMSimulation", SimulationConstructionSet2.doNothingPhysicsEngine());
      scs2.addRobot(lipm.getRobot());
      double dt = 0.001;

      // Setup controller
      ControllerDefinition controllerDefinition = LIPMDynamicsController.createControllerDefinition(dt);
      RobotControllerManager controllerManager = lipm.getRobot().getControllerManager();
      LIPMDynamicsController controller = (LIPMDynamicsController) controllerDefinition.newController(controllerManager.getControllerInput(), controllerManager.getControllerOutput());
      scs2.addYoGraphic(controller.getSCS2YoGraphics());
      lipm.getRobot().addController(controller);

      // Add ground graphic
      YoGraphicBox3DDefinition groundBox = new YoGraphicBox3DDefinition();
      groundBox.setName("groundBox");
      groundBox.setColor(ColorDefinitions.Green());
      groundBox.setPosition(YoGraphicDefinitionFactory.newYoTuple3DDefinition(0.0, 0.0, -0.5, ReferenceFrame.getWorldFrame()));
      groundBox.setSize(YoGraphicDefinitionFactory.newYoTuple3DDefinition(10.0, 10.0, 1.0, ReferenceFrame.getWorldFrame()));
      groundBox.setOrientation(YoGraphicDefinitionFactory.newYoOrientation3DDefinition(new Quaternion(), ReferenceFrame.getWorldFrame()));
      scs2.addYoGraphic(groundBox);

      scs2.setDT(dt);
      scs2.setRealTimeRateSimulation(true);

      scs2.start(true, false, false);
   }

   public static void main(String[] args)
   {
      new LIPMSimulation();
   }
}
