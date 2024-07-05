package controlsandbox.rl;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Optional;

import static controlsandbox.rl.TorqueLimitedPendulum.*;

public class TorqueLimitedPendulumValueIteration
{
   private static final String directory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "rl" + File.separator;
   private static final boolean loadFromFile = false;
   private static final boolean loadFromLatestFile = true;

   private static final double goalValue = 0.0;
   private static final double initialGuessTimeToTop = 1.5;

   private static final double rewardPerTimeStep = -0.01;
   private double[] values;
   private int[] policy;

   private long defaultIterationsPerUpdate = 1;
   private long iterationsPerUpdate = defaultIterationsPerUpdate;
   private final double[] torques;

   private final RewardHeatMapPanel rewardHeatMapPanel;

   public TorqueLimitedPendulumValueIteration()
   {
      boolean loadedFromFile = false;

      if (loadFromLatestFile)
      {
         loadedFromFile = loadFromLatestFile();
      }

      if (!loadedFromFile)
      {
         values = new double[TorqueLimitedPendulum.getStateSize()];

         for (int state_idx = 0; state_idx < values.length; state_idx++)
         {
            TorqueLimitedPendulum pendulum = new TorqueLimitedPendulum(state_idx);
            double delta_q = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(pendulum.getQ(), Math.PI));
            double timeEstimateSec = initialGuessTimeToTop * (delta_q / Math.PI);
            int timeEstimateTicks = (int) (timeEstimateSec / TorqueLimitedPendulum.DT);
            values[state_idx] = timeEstimateTicks * rewardPerTimeStep;
         }
      }

      policy = new int[TorqueLimitedPendulum.getStateSize()];
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
               else if (input.toLowerCase().equals("s"))
               {
                  saveValues();
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

   private void saveValues()
   {
      try
      {
         String fileName = directory + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()) + "_pendulumValueFunction.txt";

         FileTools.ensureDirectoryExists(new File(directory).toPath());

         File file = new File(fileName);
         FileTools.ensureFileExists(file.toPath());

         FileWriter fileWriter = new FileWriter(fileName);

         fileWriter.write("POSITION_DISCRETIZATION:" + POSITION_DISCRETIZATION + "\n");
         fileWriter.write("VELOCITY_DISCRETIZATION:" + VELOCITY_DISCRETIZATION + "\n");
         fileWriter.write("TORQUE_DISCRETIZATION:" + TORQUE_DISCRETIZATION + "\n");
         fileWriter.write("MAX_TORQUE:" + MAX_TORQUE + "\n");
         fileWriter.write("MAX_VELOCITY:" + MAX_VELOCITY + "\n");
         fileWriter.write("DT:" + DT + "\n");

         fileWriter.write("VALUES:");
         for (int i = 0; i < values.length; i++)
         {
            fileWriter.write(values[i] + ",");
         }
         fileWriter.write("\n");

         fileWriter.flush();
         fileWriter.close();
         LogTools.info("saved: " + fileName);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private boolean loadFromLatestFile()
   {
      try
      {
         Optional<Path> latestFile = Files.list(Paths.get(directory)).sorted().findFirst();
         if (!latestFile.isPresent())
         {
            LogTools.info("no file found");
            return false;
         }

         LogTools.info("Loading " + latestFile.get().toFile().getName());
         load(latestFile.get().toFile());
         return true;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return false;
      }
   }

   private void load(File file)
   {
      try
      {
         BufferedReader fileReader = new BufferedReader(new FileReader(file));
         POSITION_DISCRETIZATION = Integer.parseInt(fileReader.readLine().split(":")[1]);
         VELOCITY_DISCRETIZATION = Integer.parseInt(fileReader.readLine().split(":")[1]);
         TORQUE_DISCRETIZATION = Integer.parseInt(fileReader.readLine().split(":")[1]);
         MAX_TORQUE = Double.parseDouble(fileReader.readLine().split(":")[1]);
         MAX_VELOCITY = Double.parseDouble(fileReader.readLine().split(":")[1]);
         DT = Double.parseDouble(fileReader.readLine().split(":")[1]);
         updateGridParameters();

         String[] valuesStrings = fileReader.readLine().split(":")[1].split(",");
         values = new double[TorqueLimitedPendulum.getStateSize()];

         for (int i = 0; i < valuesStrings.length - 1; i++)
         {
            values[i] = Double.parseDouble(valuesStrings[i]);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private static void showUsage()
   {
      System.out.println("RUN VALUE ITERATION: type # of iterations (if empty will do 1), then ENTER");
      System.out.println("SAVE: type 's' then ENTER");
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

         double previewWindow = 4.0; // sec
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
