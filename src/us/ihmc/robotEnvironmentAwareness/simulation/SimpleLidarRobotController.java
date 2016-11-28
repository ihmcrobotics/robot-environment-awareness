package us.ihmc.robotEnvironmentAwareness.simulation;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.graphics3DAdapter.GPULidar;
import us.ihmc.graphics3DAdapter.GPULidarScanBuffer;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPlanarRegionsList;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarScanMessage;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.tools.thread.ThreadTools;

public class SimpleLidarRobotController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("Messaging"));

   private final BooleanYoVariable spinLidar = new BooleanYoVariable("spinLidar", registry);
   private final DoubleYoVariable desiredLidarVelocity = new DoubleYoVariable("desiredLidarVelocity", registry);
   private final DoubleYoVariable lidarRange = new DoubleYoVariable("lidarRange", registry);

   private final PinJoint lidarJoint;
   private final double dt;
   private final FloatingJoint rootJoint;

   private final LidarScanParameters lidarScanParameters;
   private final GPULidar gpuLidar;
   private final GPULidarScanBuffer gpuLidarScanBuffer;
   private final int vizualizeEveryNPoints = 5;
   private final BagOfBalls sweepViz;

   private final PacketCommunicator packetCommunicator;

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("region", 100, 150, registry);

   public SimpleLidarRobotController(SimpleLidarRobot lidarRobot, double dt, PacketCommunicator packetCommunicator, Graphics3DAdapter graphics3dAdapter,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.dt = dt;
      this.packetCommunicator = packetCommunicator;
      lidarJoint = lidarRobot.getLidarJoint();
      rootJoint = lidarRobot.getRootJoint();

      desiredLidarVelocity.set(LidarFastSimulation.DEFAULT_SPIN_VELOCITY);
      spinLidar.set(true);
      lidarRange.set(30.0);

      final YoFrameOrientation lidarYawPitchRoll = new YoFrameOrientation("lidar", null, registry);
      lidarYawPitchRoll.attachVariableChangedListener(new VariableChangedListener()
      {
         private final Quat4d localQuaternion = new Quat4d();

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            lidarYawPitchRoll.getQuaternion(localQuaternion);
            rootJoint.setQuaternion(localQuaternion);
         }
      });

      lidarScanParameters = lidarRobot.getLidarScanParameters();
      gpuLidarScanBuffer = new GPULidarScanBuffer(lidarScanParameters);
      gpuLidar = graphics3dAdapter.createGPULidar(gpuLidarScanBuffer, lidarScanParameters);
      if (LidarFastSimulation.VISUALIZE_GPU_LIDAR)
         sweepViz = BagOfBalls.createRainbowBag(lidarScanParameters.getPointsPerSweep() / vizualizeEveryNPoints, 0.005, "SweepViz", registry, yoGraphicsListRegistry);
      else
         sweepViz = null;
      yoGraphicPlanarRegionsList.hideGraphicObject();
      yoGraphicsListRegistry.registerYoGraphic("Regions", yoGraphicPlanarRegionsList);
      packetCommunicator.attachListener(PlanarRegionsListMessage.class, this::handlePacket);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      if (spinLidar.getBooleanValue())
      {
         lidarJoint.getQYoVariable().add(desiredLidarVelocity.getDoubleValue() * dt);
      }

      RigidBodyTransform transform = new RigidBodyTransform();
      Point3f lidarPosition = new Point3f();
      Quat4f lidarOrientation = new Quat4f();

      lidarJoint.getTransformToWorld(transform);
      transform.get(lidarOrientation, lidarPosition);

      gpuLidar.setTransformFromWorld(transform, 0);

      while (!gpuLidarScanBuffer.isEmpty())
      {
         LidarScan scan = gpuLidarScanBuffer.poll();

         if (LidarFastSimulation.VISUALIZE_GPU_LIDAR)
         {
            for (int i = 0; i < scan.size(); i += vizualizeEveryNPoints)
            {
               sweepViz.setBallLoop(new FramePoint(ReferenceFrame.getWorldFrame(), scan.getPoint(i)));
            }
         }

         TFloatArrayList newScan = new TFloatArrayList();
         for (int i = 0; i < scan.size(); i++)
         {
            Point3d sensorOrigin = new Point3d();
            transform.getTranslation(sensorOrigin);
            Point3d scanPoint = scan.getPoint(i);
            if (sensorOrigin.distance(scanPoint) < lidarRange.getDoubleValue())
            {
               newScan.add((float) scanPoint.getX());
               newScan.add((float) scanPoint.getY());
               newScan.add((float) scanPoint.getZ());
            }
         }

         LidarScanMessage lidarScanMessage = new LidarScanMessage(-1L, lidarPosition, lidarOrientation, newScan.toArray());
         executorService.execute(() -> packetCommunicator.send(lidarScanMessage));
      }

      packetCommunicator.send(new RequestPlanarRegionsListMessage(RequestType.CONTINUOUS_UPDATE));
      yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
   }

   public void handlePacket(PlanarRegionsListMessage message)
   {
      if (message != null)
         yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(PlanarRegionMessageConverter.convertToPlanarRegionsList(message));
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}
