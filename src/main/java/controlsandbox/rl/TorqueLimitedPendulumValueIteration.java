package controlsandbox.rl;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;

import static controlsandbox.rl.TorqueLimitedPendulum.*;

public class TorqueLimitedPendulumValueIteration
{
   private static final double goalValue = 0.0;
   private static final double initialValues = -1.0;

   private static final double rewardPerTimeStep = -0.01;
   private double[] values;
   private int[] policy;

   private long defaultIterationsPerUpdate = 1;
   private long iterationsPerUpdate = defaultIterationsPerUpdate;
   private final double[] torques;

   private final RewardHeatMapPanel rewardHeatMapPanel;

   public TorqueLimitedPendulumValueIteration()
   {
      values = new double[TorqueLimitedPendulum.getStateSize()];
      policy = new int[TorqueLimitedPendulum.getStateSize()];

      Arrays.fill(values, initialValues);
      Arrays.fill(policy, TORQUE_DISCRETIZATION);

      rewardHeatMapPanel = new RewardHeatMapPanel(POSITION_DISCRETIZATION, 2 * VELOCITY_DISCRETIZATION + 1);

      torques = new double[2 * TORQUE_DISCRETIZATION + 1];
      for (int i = 0; i < torques.length; i++)
      {
//         torques[i] = MAX_TORQUE * EuclidCoreTools.square((i - TORQUE_DISCRETIZATION) / (double) TORQUE_DISCRETIZATION);
         torques[i] = MAX_TORQUE * ((i - TORQUE_DISCRETIZATION) / (double) TORQUE_DISCRETIZATION);
      }

      start();
   }

   private void start()
   {
      boolean exitRequested = false;
      showDisplay();
      showUsage();

      while (!exitRequested)
      {
         doValueIterationUpdate();

         System.out.println("finished");

         while (true)
         {
            try
            {
               BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
               String input = reader.readLine();

               if (input.toLowerCase().equals("e"))
               {
                  exitRequested = true;
               }

               try
               {
                  iterationsPerUpdate = Integer.parseInt(input);
               }
               catch (NumberFormatException e)
               {
                  iterationsPerUpdate = defaultIterationsPerUpdate;
               }

               break;
            }
            catch (IOException e)
            {
               exitRequested = true;
               break;
            }
         }
      }
   }

   public void doValueIterationUpdate()
   {
      int iterationCount = 0;
      boolean valueUpdated = false;

      while (iterationCount < iterationsPerUpdate)
      {
         iterationCount++;

         for (int stateToUpdate = 0; stateToUpdate < values.length; stateToUpdate++)
         {
            // do value iteration update

            TorqueLimitedPendulum pendulumState = new TorqueLimitedPendulum(stateToUpdate);
            if (pendulumState.isGoalState())
            {
               values[stateToUpdate] = goalValue;
               continue;
            }

            double maxCandidateValue = Double.NEGATIVE_INFINITY;
            int maxValueAction = -1;

            double originalValue = values[stateToUpdate];

            for (int action_idx = 0; action_idx < torques.length; action_idx++)
            {
               double torque = torques[action_idx];

               TorqueLimitedPendulum pendulumStateToUpdateFrom = new TorqueLimitedPendulum(stateToUpdate);
               pendulumStateToUpdateFrom.doSimulationForwardStep(torque);
               int stateToUpdateFrom = pendulumStateToUpdateFrom.getStateIndex();

               double candidateValue = rewardPerTimeStep + values[stateToUpdateFrom];
               if (candidateValue > maxCandidateValue)
               {
                  maxCandidateValue = candidateValue;
                  maxValueAction = action_idx;
               }
            }

            if (!EuclidCoreTools.epsilonEquals(originalValue, maxCandidateValue, 1e-6))
            {
               valueUpdated = true;
            }

            values[stateToUpdate] = maxCandidateValue;
            policy[stateToUpdate] = maxValueAction;
         }

         System.out.print(iterationCount + ",");

         rewardHeatMapPanel.update(values, policy, torques);
         rewardHeatMapPanel.repaint();
      }

      if (!valueUpdated)
      {
         LogTools.info("no values updated");
      }
   }

   private static void showUsage()
   {
      System.out.println("Type # of value iterations (if empty will do 1), then ENTER");
      System.out.println("To save policy, type 's' then ENTER");
   }

   private void showDisplay()
   {
      EventQueue.invokeLater(() ->
                             {
                                try
                                {
                                   UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
                                }
                                catch (ClassNotFoundException | InstantiationException | IllegalAccessException | UnsupportedLookAndFeelException ex)
                                {
                                }

                                JFrame frame = new JFrame("Pendulum Value Function");
                                frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
                                frame.setLayout(new BorderLayout());
                                frame.add(rewardHeatMapPanel);
                                frame.pack();
                                frame.setLocationRelativeTo(null);
                                frame.setVisible(true);
                             });
   }

   public static class RewardHeatMapPanel extends JPanel
   {
      private final BufferedImage rewardImage;
      private final int pixelsPerStateQ = 3;
      private final int pixelsPerStateQd = 1;

      public RewardHeatMapPanel(int qNum, int qdNum)
      {
         rewardImage = new BufferedImage(pixelsPerStateQ * qNum, pixelsPerStateQd * qdNum, BufferedImage.TYPE_INT_RGB);

         for (int i = 0; i < rewardImage.getWidth(); i++)
         {
            for (int j = 0; j < rewardImage.getHeight(); j++)
            {
               rewardImage.setRGB(i, j, Color.BLACK.getRGB());
            }
         }
      }

      public void update(double[] values, int[] policy, double[] torques)
      {
         double minValue = 0.0;
         for (int i = 0; i < values.length; i++)
         {
            if (Double.isNaN(values[i]))
               continue;
            minValue = Math.min(values[i], minValue);
         }

         for (int stateIndex = 0; stateIndex < values.length; stateIndex++)
         {
            TorqueLimitedPendulum pendulum = new TorqueLimitedPendulum(stateIndex);

            Color color;

            if (Double.isNaN(values[stateIndex]))
            {
               color = Color.BLACK;
            }
            else
            {
               int c = (int) (255 * (values[stateIndex] - minValue) / Math.abs(minValue));
               color = new Color(c, c, c);
            }

            setColorFromState(pendulum, color);
         }

         double previewWindow = 2.5; // sec
         int previewSteps = (int) (previewWindow / DT);
         TorqueLimitedPendulum pendulum = new TorqueLimitedPendulum(0.0, 0.0);

         for (int i = 0; i < previewSteps; i++)
         {
            setColorFromState(pendulum, Color.RED);
            pendulum.doSimulationForwardStep(torques[policy[pendulum.getStateIndex()]]);
         }
      }

      private void setColorFromState(TorqueLimitedPendulum pendulum, Color color)
      {
         int x = pixelsPerStateQ * pendulum.getPositionIndex();
         int y = pixelsPerStateQd * (-pendulum.getVelocityIndex() + VELOCITY_DISCRETIZATION);

         for (int xiOffset = 0; xiOffset < pixelsPerStateQ; xiOffset++)
         {
            for (int yiOffset = 0; yiOffset < pixelsPerStateQd; yiOffset++)
            {
               rewardImage.setRGB(x + xiOffset, y + yiOffset, color.getRGB());
            }
         }
      }

      @Override
      public Dimension getPreferredSize()
      {
         return new Dimension(rewardImage.getWidth(), rewardImage.getHeight());
      }

      @Override
      protected void paintComponent(Graphics g)
      {
         super.paintComponent(g);
         Graphics2D g2d = (Graphics2D) g.create();
         g2d.drawImage(rewardImage, 0, 0, this);
         g2d.dispose();
      }
   }

   public static void main(String[] args)
   {
      new TorqueLimitedPendulumValueIteration();
   }
}
