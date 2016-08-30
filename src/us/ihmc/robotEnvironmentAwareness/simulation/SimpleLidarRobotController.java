package us.ihmc.robotEnvironmentAwareness.simulation;

import java.util.concurrent.locks.ReentrantReadWriteLock;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.graphics3DAdapter.GPULidar;
import us.ihmc.graphics3DAdapter.GPULidarScanBuffer;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;
import us.ihmc.ihmcPerception.depthData.PointCloudWorldPacketGenerator;
import us.ihmc.robotEnvironmentAwareness.communication.LidarPosePacket;
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
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class SimpleLidarRobotController implements RobotController
{
   private static final double DEFAULT_SPIN_VELOCITY = 0.3;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable spinLidar = new BooleanYoVariable("spinLidar", registry);
   private final DoubleYoVariable desiredLidarVelocity = new DoubleYoVariable("desiredLidarVelocity", registry);
   private final PinJoint lidarJoint;
   private final double dt;
   private final FloatingJoint rootJoint;

   private final LidarScanParameters lidarScanParameters;
   private final GPULidar gpuLidar;
   private final GPULidarScanBuffer gpuLidarScanBuffer;
   private final int vizualizeEveryNPoints = 5;
   private final BagOfBalls sweepViz;

   private final ReentrantReadWriteLock readWriteLock = new ReentrantReadWriteLock();
   private final DepthDataFilter depthDataFilter = new DepthDataFilter();
   private final PointCloudWorldPacketGenerator pointCloudWorldPacketGenerator;

   private final PacketCommunicator packetCommunicator;

   public SimpleLidarRobotController(SimpleLidarRobot lidarRobot, double dt, PacketCommunicator packetCommunicator, Graphics3DAdapter graphics3dAdapter,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.dt = dt;
      this.packetCommunicator = packetCommunicator;
      lidarJoint = lidarRobot.getLidarJoint();
      rootJoint = lidarRobot.getRootJoint();

      desiredLidarVelocity.set(DEFAULT_SPIN_VELOCITY);
      spinLidar.set(true);

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
      sweepViz = BagOfBalls.createRainbowBag(lidarScanParameters.getPointsPerSweep() / vizualizeEveryNPoints, 0.005, "SweepViz", registry, yoGraphicsListRegistry);

      pointCloudWorldPacketGenerator = new PointCloudWorldPacketGenerator(packetCommunicator, readWriteLock.readLock(), depthDataFilter);
      pointCloudWorldPacketGenerator.start();
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
         lidarJoint.getQ().add(desiredLidarVelocity.getDoubleValue() * dt);
      }

      RigidBodyTransform transform = new RigidBodyTransform();

      lidarJoint.getTransformToWorld(transform);

      gpuLidar.setTransformFromWorld(transform, 0);

      while (!gpuLidarScanBuffer.isEmpty())
      {
         LidarScan scan = gpuLidarScanBuffer.poll();

         for (int i = 0; i < scan.size(); i += vizualizeEveryNPoints)
         {
            sweepViz.setBallLoop(new FramePoint(ReferenceFrame.getWorldFrame(), scan.getPoint(i)));
         }

         readWriteLock.writeLock().lock();
         try
         {
            for (int i = 0; i < scan.size(); i++)
            {
               Point3d sensorOrigin = new Point3d();
               transform.getTranslation(sensorOrigin);
               depthDataFilter.addPoint(scan.getPoint(i), sensorOrigin);
            }
         }
         finally
         {
            readWriteLock.writeLock().unlock();
         }
      }

      packetCommunicator.send(generateLidarPosePacket());
   }

   private LidarPosePacket generateLidarPosePacket()
   {
      Point3d position = new Point3d();
      Quat4d orientation = new Quat4d();
      RigidBodyTransform transform = new RigidBodyTransform();
      lidarJoint.getTransformToWorld(transform);
      transform.get(orientation, position);
      LidarPosePacket lidarPosePacket = new LidarPosePacket(position, orientation);
      lidarPosePacket.setLidarAngleJoint((float) lidarJoint.getQ().getDoubleValue());
      return lidarPosePacket;
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
