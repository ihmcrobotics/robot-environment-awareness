package us.ihmc.robotEnvironmentAwareness.simulation;

import java.io.IOException;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.FlatGroundEnvironment;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.robotEnvironmentAwareness.communication.LidarSimulationNetClassList;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.util.RealtimeTools;

public class FootstepPlanningFastSimulation
{
   private enum GroundType
   {
      OBSTACLE_COURSE, FLAT
   };

   public FootstepPlanningFastSimulation() throws IOException
   {
      SimpleLidarRobot robot = new SimpleLidarRobot();
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, RealtimeTools.nextPowerOfTwo(200000));
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(0.0001, 10);

      double dt = scs.getDT();
      Graphics3DAdapter graphics3dAdapter = scs.getGraphics3dAdapter();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.BEHAVIOUR_MODULE_PORT, new LidarSimulationNetClassList());
      packetCommunicator.connect();

      SimpleLidarRobotController controller = new SimpleLidarRobotController(robot, dt, packetCommunicator, graphics3dAdapter, yoGraphicsListRegistry);
      robot.setController(controller);
//      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      createGroundTypeListener(scs);

      scs.setGroundVisible(false);
      scs.startOnAThread();
   }

   private void createGroundTypeListener(final SimulationConstructionSet scs)
   {
      final EnumYoVariable<GroundType> groundType = new EnumYoVariable<>("GroundType", scs.getRootRegistry(), GroundType.class, false);

      VariableChangedListener listener = new VariableChangedListener()
      {
         private Graphics3DNode groundGraphicsNode = null;

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (groundGraphicsNode != null)
               scs.removeGraphics3dNode(groundGraphicsNode);

            CommonAvatarEnvironmentInterface environment = null;

            switch (groundType.getEnumValue())
            {
            case OBSTACLE_COURSE:
               environment = new DRCDemo01NavigationEnvironment();
               break;
            case FLAT:
               environment = new FlatGroundEnvironment();
               break;
            }
            groundGraphicsNode = scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());
         }
      };
      groundType.addVariableChangedListener(listener);
      listener.variableChanged(null);
   }

   public static void main(String[] args) throws IOException
   {
      new FootstepPlanningFastSimulation();
   }
}
