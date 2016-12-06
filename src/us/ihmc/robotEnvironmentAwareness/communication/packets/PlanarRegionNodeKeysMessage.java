package us.ihmc.robotEnvironmentAwareness.communication.packets;

import java.util.Arrays;

import us.ihmc.communication.packets.Packet;
import us.ihmc.jOctoMap.key.OcTreeKey;

public class PlanarRegionNodeKeysMessage extends Packet<PlanarRegionNodeKeysMessage>
{
   public int id;
   public OcTreeKeyMessage[] nodeKeys;

   public PlanarRegionNodeKeysMessage()
   {
   }

   public PlanarRegionNodeKeysMessage(int id, OcTreeKeyMessage[] regionNodeKeys)
   {
      this.id = id;
      this.nodeKeys = regionNodeKeys;
   }

   public int getRegionId()
   {
      return id;
   }

   public int getNumberOfNodes()
   {
      return nodeKeys.length;
   }

   public OcTreeKeyMessage getNodeKey(int index)
   {
      return nodeKeys[index];
   }

   public void getNodeKey(int index, OcTreeKey nodeKeyToPack)
   {
      nodeKeyToPack.set(nodeKeys[index]);
   }

   @Override
   public boolean epsilonEquals(PlanarRegionNodeKeysMessage other, double epsilon)
   {
      return id == other.id && Arrays.equals(nodeKeys, other.nodeKeys);
   }
}
