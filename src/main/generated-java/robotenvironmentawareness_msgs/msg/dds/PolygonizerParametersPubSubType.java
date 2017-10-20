package robotenvironmentawareness_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PolygonizerParameters" defined in "PolygonizerParameters_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PolygonizerParameters_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PolygonizerParameters_.idl instead.
*
*/
public class PolygonizerParametersPubSubType implements us.ihmc.pubsub.TopicDataType<robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters>
{
	public static final java.lang.String name = "robotenvironmentawareness_msgs::msg::dds_::PolygonizerParameters_";
	
	
	
    public PolygonizerParametersPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }
   
	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_6(data.getConcave_hull_threshold());

	    cdr.write_type_4(data.getMin_number_of_nodes());

	    cdr.write_type_6(data.getShallow_angle_threshold());

	    cdr.write_type_6(data.getPeak_angle_threshold());

	    cdr.write_type_6(data.getLength_threshold());

	    cdr.write_type_6(data.getDepth_threshold());
   }

   public static void read(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data, us.ihmc.idl.CDR cdr)
   {

	    	data.setConcave_hull_threshold(cdr.read_type_6());
	    	

	    	data.setMin_number_of_nodes(cdr.read_type_4());
	    	

	    	data.setShallow_angle_threshold(cdr.read_type_6());
	    	

	    	data.setPeak_angle_threshold(cdr.read_type_6());
	    	

	    	data.setLength_threshold(cdr.read_type_6());
	    	

	    	data.setDepth_threshold(cdr.read_type_6());
	    	
   }
   
	@Override
	public final void serialize(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_6("concave_hull_threshold", data.getConcave_hull_threshold());
			    
			    ser.write_type_4("min_number_of_nodes", data.getMin_number_of_nodes());
			    
			    ser.write_type_6("shallow_angle_threshold", data.getShallow_angle_threshold());
			    
			    ser.write_type_6("peak_angle_threshold", data.getPeak_angle_threshold());
			    
			    ser.write_type_6("length_threshold", data.getLength_threshold());
			    
			    ser.write_type_6("depth_threshold", data.getDepth_threshold());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data)
	{
	    			data.setConcave_hull_threshold(ser.read_type_6("concave_hull_threshold"));	
	    	    
	    			data.setMin_number_of_nodes(ser.read_type_4("min_number_of_nodes"));	
	    	    
	    			data.setShallow_angle_threshold(ser.read_type_6("shallow_angle_threshold"));	
	    	    
	    			data.setPeak_angle_threshold(ser.read_type_6("peak_angle_threshold"));	
	    	    
	    			data.setLength_threshold(ser.read_type_6("length_threshold"));	
	    	    
	    			data.setDepth_threshold(ser.read_type_6("depth_threshold"));	
	    	    
	}

   public static void staticCopy(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters src, robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters createData()
   {
      return new robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters();
   }
      

   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters src, robotenvironmentawareness_msgs.msg.dds.PolygonizerParameters dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public PolygonizerParametersPubSubType newInstance()
   {
   	  return new PolygonizerParametersPubSubType();
   }
}