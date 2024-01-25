package controlsandbox.cartPole;

import controlsandbox.lqr.LQRController;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class CartPoleSimulation
{
   public CartPoleSimulation()
   {
      CartPoleRobot cartPole = new CartPoleRobot();

      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2("CartPoleSimulation");
      scs2.addRobot(cartPole.getRobot());

      cartPole.getPoleJoint().setQ(Math.PI - 0.02);

      cartPole.getRobot().addController(LQRController.setupForCartPole(cartPole));
      scs2.setDT(0.0001);
      scs2.setRealTimeRateSimulation(true);

      scs2.start(true, false, false);
   }

   public static void main(String[] args)
   {
      new CartPoleSimulation();
   }
}
