package controlsandbox.rl;

import us.ihmc.euclid.tools.EuclidCoreTools;

public class TorqueLimitedPendulum
{
   public static final double Q_GOAL = Math.PI;
   public static final double QD_GOAL = 0.0;
   public static final double GOAL_Q_EPSILON = Math.toRadians(1.5);
   public static final double GOAL_QD_EPSILON = Math.toRadians(15.0);

   public static final double LENGTH = 1.0;
   public static final double G = 9.81;
   public static final double MASS = 5.0;

   // Configurable parameters
   public static int POSITION_DISCRETIZATION = 720;
   public static int VELOCITY_DISCRETIZATION = 500;
   public static int TORQUE_DISCRETIZATION = 12;

   public static double MAX_TORQUE = 0.4 * MASS * G * LENGTH;
   public static double MAX_VELOCITY = 6.4;
   public static double DT = 0.004;

   // Grid parameters
   public static double POSITION_GRID_SIZE;
   public static double VELOCITY_GRID_SIZE;

   public static void updateGridParameters()
   {
      POSITION_GRID_SIZE = 2.0 * Math.PI / POSITION_DISCRETIZATION;
      VELOCITY_GRID_SIZE = MAX_VELOCITY / VELOCITY_DISCRETIZATION;
   }

   static
   {
      updateGridParameters();
   }

   /* Index from 0 -> (POSITION_DISCRETIZATION - 1), where 0 represents angle = 0rad */
   private int q_i;
   /* Index from -VELOCITY_DISCRETIZATION -> VELOCITY_DISCRETIZATION, where 0 represents 0 velocity */
   private int qd_i;

   /* Pendulum angle in radians */
   private double q;
   /* Pendulum angular velocity in radians/sec */
   private double qd;

   /* Single zero-index integer value representing the state */
   private int stateIndex;

   public TorqueLimitedPendulum(int stateIndex)
   {
      this.q_i = positionFromStateIndex(stateIndex);
      this.qd_i = velocityFromStateIndex(stateIndex);
      updateStateIndex();

      updateCoordinates();
   }

   public TorqueLimitedPendulum(int q_i0, int q_id0)
   {
      this.q_i = q_i0;
      this.qd_i = q_id0;

      updateCoordinates();
   }

   public TorqueLimitedPendulum(double q0, double qd0)
   {
      this.q = q0;
      this.qd = qd0;

      updateIndices();
   }

   public void doSimulationForwardStep(double tau)
   {
      double qdd = -G * Math.sin(q) / LENGTH + tau / (MASS * LENGTH);
      double qd1 = qd + qdd * DT;
      q += 0.5 * (qd + qd1) * DT;
      qd = qd1;

      updateIndices();
   }

   public boolean isGoalState()
   {
      return EuclidCoreTools.epsilonEquals(q, Q_GOAL, GOAL_Q_EPSILON) && EuclidCoreTools.epsilonEquals(qd, QD_GOAL, GOAL_QD_EPSILON);
   }

   public double getQ()
   {
      return q;
   }

   public double getQd()
   {
      return qd;
   }

   /* Zero-indexed value of the state */
   public int getStateIndex()
   {
      return stateIndex;
   }

   private void updateIndices()
   {
      q_i = positionToIndex(q);
      qd_i = velocityToIndex(qd);
      updateStateIndex();
      updateCoordinates();
   }

   private void updateStateIndex()
   {
      stateIndex = toStateIndex(q_i, qd_i);
   }

   private static int positionFromStateIndex(int stateIndex)
   {
      return stateIndex / (2 * VELOCITY_DISCRETIZATION + 1);
   }

   private static int velocityFromStateIndex(int stateIndex)
   {
      return (stateIndex % (2 * VELOCITY_DISCRETIZATION + 1)) - VELOCITY_DISCRETIZATION;
   }

   public static int getStateSize()
   {
      return POSITION_DISCRETIZATION * (2 * VELOCITY_DISCRETIZATION + 1);
   }

   private static int toStateIndex(int q_i, int qd_i)
   {
      int numVelocityStates = 2 * VELOCITY_DISCRETIZATION + 1;
      return numVelocityStates * q_i + (qd_i + VELOCITY_DISCRETIZATION);
   }

   private void updateCoordinates()
   {
      q = positionFromIndex(q_i);
      qd = velocityFromIndex(qd_i);
   }

   public int getPositionIndex()
   {
      return q_i;
   }

   public int getVelocityIndex()
   {
      return qd_i;
   }

   private static int positionToIndex(double q)
   {
      q = EuclidCoreTools.shiftAngleInRange(q, 0.0);
      return Math.floorMod((int) (Math.round(q / POSITION_GRID_SIZE)), POSITION_DISCRETIZATION);
   }

   private static double positionFromIndex(int qIndex)
   {
      return qIndex * POSITION_GRID_SIZE;
   }

   private static int velocityToIndex(double qd)
   {
      qd = EuclidCoreTools.clamp(qd, MAX_VELOCITY);
      return (int) Math.round(qd * VELOCITY_DISCRETIZATION / MAX_VELOCITY);
   }

   private static double velocityFromIndex(int qdIndex)
   {
      return qdIndex * VELOCITY_GRID_SIZE;
   }

   public static void main(String[] args)
   {
//      convTest(0.49);
//      convTest(0.51);
//      convTest(1.49);
//      convTest(355.4);
//      convTest(360.0);

//      TorqueLimitedPendulum pendulum = new TorqueLimitedPendulum(3209);
//      System.out.println(pendulum.getStateIndex());

//      System.out.println(positionFromIndex(45));

      TorqueLimitedPendulum pendulum = new TorqueLimitedPendulum(0.0, 2.0 * Math.sqrt(G / LENGTH));
      for (int i = 0; i < 300; i++)
      {
         pendulum.doSimulationForwardStep(0.0);
         if (i % 1 == 0)
            System.out.println(pendulum.getQ() + ", " + pendulum.getQd());
      }
   }

   private static void convTest(double q0Deg)
   {
      System.out.println("q degrees: " + q0Deg);
      double q0Rad = Math.toRadians(q0Deg);
      int q0Index = positionToIndex(q0Rad);
      System.out.println(q0Index);
      double q0Rad2 = positionFromIndex(q0Index);
      double q0Deg2 = Math.toDegrees(q0Rad2);
      System.out.println(q0Deg2);
      System.out.println("----------------------------");
   }
}
