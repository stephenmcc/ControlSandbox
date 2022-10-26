package controlsandbox.cartPole;

import controlsandbox.solver.LQRController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class CartPoleSimulation
{
   public CartPoleSimulation()
   {
      CartPoleRobot robot = new CartPoleRobot();
      robot.getSCSPoleJoint().setQ(Math.PI);
      robot.getRobot().setController(LQRController.setupForCartPole(robot));

      SimulationConstructionSet scs = new SimulationConstructionSet(robot.getRobot());

      scs.setGroundVisible(false);
      scs.setSimulateNoFasterThanRealTime(true);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new CartPoleSimulation();
   }
}
