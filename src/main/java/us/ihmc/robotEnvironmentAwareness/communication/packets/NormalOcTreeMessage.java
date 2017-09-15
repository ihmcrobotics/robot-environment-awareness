package us.ihmc.robotEnvironmentAwareness.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.MathTools;

public class NormalOcTreeMessage extends Packet<NormalOcTreeMessage>
{
   public int treeDepth = 0;
   public float resolution = Float.NaN;
   public NormalOcTreeNodeMessage root = null;

   public NormalOcTreeMessage()
   {
   }

   @Override
   public boolean epsilonEquals(NormalOcTreeMessage other, double epsilon)
   {
      if (treeDepth != other.treeDepth)
         return false;
      if (!MathTools.epsilonEquals(resolution, other.resolution, epsilon))
         return false;
      if (!root.epsilonEquals(other.root, epsilon))
         return false;
      return true;
   }
}
