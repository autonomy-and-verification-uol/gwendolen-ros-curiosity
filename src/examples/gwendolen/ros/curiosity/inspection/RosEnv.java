package gwendolen.ros.curiosity.inspection;


import com.fasterxml.jackson.databind.JsonNode;

import ail.mas.DefaultEnvironment;
import ail.syntax.Action;
import ail.syntax.Literal;
import ail.syntax.NumberTerm;
import ail.syntax.NumberTermImpl;
import ail.syntax.StringTerm;
import ail.syntax.StringTermImpl;
import ail.syntax.Unifier;
import ail.util.AILexception;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.geometry_msgs.Twist;
import ros.msgs.geometry_msgs.Vector3;
import ros.msgs.move_base_msgs.MoveBaseActionResult;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;
import ros.tools.PeriodicPublisher;



public class RosEnv extends DefaultEnvironment{
	static final String logname = "gwendolen.ros.curiosity.inspection.RosEnv";

	RosBridge bridge = new RosBridge();


	/**
	 * Constructor.  Decides upon the number of humans, buildings and rubble.
	 */
	public RosEnv() {
		super();

		bridge.connect("ws://localhost:9090", true);
		System.out.println("Environment started, connection with ROS established.");

		bridge.subscribe(SubscriptionRequestMsg.generate("/curiosity_to_agent")
				.setType("std_msgs/String")
//				.setThrottleRate(1)
				.setQueueLength(4),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(PrimitiveMsg.class);
					PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
					clearPercepts();
//					System.out.println("Message: "+msg.data);
//					System.out.println("Frame id: "+msg.header.frame_id);
//					System.out.println("Stamp sec: "+msg.header.stamp.secs);
//					System.out.println("Seq: "+msg.header.seq);
//					System.out.println("Goal: "+msg.status.goal_id.id);
//					System.out.println("Stamp sec: "+msg.status.goal_id.stamp.secs);
//					System.out.println("Status: "+msg.status.status);
//					System.out.println("Text: "+msg.status.text);
//
//					System.out.println();
					if (msg.data.equals("done")) {
						Literal movement_completed = new Literal("movement_completed");
						//movement_completed.addTerm(new StringTermImpl(msg.data));
						addPercept(movement_completed);
					} else if (msg.data.startsWith("mast(")) {
						Literal mast_result = new Literal("mast");
						System.out.println(msg.data);
						mast_result.addTerm(new StringTermImpl(msg.data.substring(5, msg.data.length()-1)));
						//movement_completed.addTerm(new StringTermImpl(msg.data));
						addPercept(mast_result);
					}  else if (msg.data.startsWith("arm(")) {
						System.out.println(msg.data);
						Literal arm_result = new Literal("arm");
						arm_result.addTerm(new StringTermImpl(msg.data.substring(4, msg.data.length()-1)));
						//movement_completed.addTerm(new StringTermImpl(msg.data));
						addPercept(arm_result);
					}
					else {
						Literal actuator_ready = new Literal("actuator_ready");
						actuator_ready.addTerm(new StringTermImpl(msg.data));
						addPercept(actuator_ready);
					}
				}
			}
	    );

		bridge.subscribe(SubscriptionRequestMsg.generate("/move_base/result")
				.setType("move_base_msgs/MoveBaseActionResult"),
//				.setThrottleRate(1)
//				.setQueueLength(1),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<MoveBaseActionResult> unpacker = new MessageUnpacker<MoveBaseActionResult>(MoveBaseActionResult.class);
					MoveBaseActionResult msg = unpacker.unpackRosMessage(data);
					clearPercepts();
//					System.out.println("Frame id: "+msg.header.frame_id);
//					System.out.println("Stamp sec: "+msg.header.stamp.secs);
//					System.out.println("Seq: "+msg.header.seq);
//					System.out.println("Goal: "+msg.status.goal_id.id);
//					System.out.println("Stamp sec: "+msg.status.goal_id.stamp.secs);
//					System.out.println("Status: "+msg.status.status);
//					System.out.println("Text: "+msg.status.text);
//
//					System.out.println();
					Literal movebase_result = new Literal("movebase_result");
					movebase_result.addTerm(new NumberTermImpl(msg.header.seq));
					movebase_result.addTerm(new NumberTermImpl(msg.status.status));
					addPercept(movebase_result);
				}
			}
	    );

	}

	/*
	 * (non-Javadoc)
	 * @see ail.mas.DefaultEnvironment#executeAction(java.lang.String, ail.syntax.Action)
	 */
	public Unifier executeAction(String agName, Action act) throws AILexception {
		String actionname = act.getFunctor();
		int nterms = act.getTermsSize();
		if (actionname.equals("hello_ros")) {
			hello_ros();
		} else if ((actionname.equals("move")) && nterms == 6) {
			NumberTerm lx = (NumberTerm) act.getTerm(0);
			NumberTerm ly = (NumberTerm) act.getTerm(1);
			NumberTerm lz = (NumberTerm) act.getTerm(2);
			NumberTerm ax = (NumberTerm) act.getTerm(3);
			NumberTerm ay = (NumberTerm) act.getTerm(4);
			NumberTerm az = (NumberTerm) act.getTerm(5);
			move(lx.solve(),ly.solve(),lz.solve(),ax.solve(),ay.solve(),az.solve());
		} else if ((actionname.equals("move")) && nterms == 3) {
			NumberTerm lx = (NumberTerm) act.getTerm(0);
			NumberTerm ly = (NumberTerm) act.getTerm(1);
			NumberTerm lz = (NumberTerm) act.getTerm(2);
			move(lx.solve(),ly.solve(),lz.solve());
		} else if (actionname.equals("keep_moving")) {
			NumberTerm period = (NumberTerm) act.getTerm(0);
			NumberTerm lx = (NumberTerm) act.getTerm(1);
			NumberTerm ly = (NumberTerm) act.getTerm(2);
			NumberTerm lz = (NumberTerm) act.getTerm(3);
			NumberTerm ax = (NumberTerm) act.getTerm(4);
			NumberTerm ay = (NumberTerm) act.getTerm(5);
			NumberTerm az = (NumberTerm) act.getTerm(6);
			keep_moving((int) period.solve(),lx.solve(),ly.solve(),lz.solve(),ax.solve(),ay.solve(),az.solve());
		} else if (actionname.equals("control_arm")) {
			StringTerm modereq = (StringTerm) act.getTerm(0);
			control_arm(modereq.getString());
		} else if (actionname.equals("control_mast")) {
			StringTerm modereq = (StringTerm) act.getTerm(0);
			control_mast(modereq.getString());
		} else if (actionname.equals("control_wheels")) {
			StringTerm modereq = (StringTerm) act.getTerm(0);
			NumberTerm distancereq = (NumberTerm) act.getTerm(1);
			control_wheels(modereq.getString(),(int) distancereq.solve());
		} else if (actionname.equals("stop_moving")) {
			stop_moving();
		} else if (actionname.equals("wait")) {
			NumberTerm period = (NumberTerm) act.getTerm(0);
			try {
				Thread.sleep((int) period.solve());
			} catch (InterruptedException e) {
			e.printStackTrace();
			}
		}

		return super.executeAction(agName, act);
	}

	public void hello_ros() {
		Publisher pub = new Publisher("/java_to_ros", "std_msgs/String", bridge);

		for(int i = 0; i < 100; i++) {
			pub.publish(new PrimitiveMsg<String>("hello from gwendolen " + i));
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	public void move(double lx, double ly, double lz, double ax, double ay, double az) {
		Publisher cmd_vel = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);

		Vector3 linear = new Vector3(lx,ly,lz);
		Vector3 angular = new Vector3(ax,ay,az);
		cmd_vel.publish(new Twist(linear, angular));
	}

	public void move(double lx, double ly, double lz) {
		Publisher move_base = new Publisher("/gwendolen_to_move_base", "geometry_msgs/Vector3", bridge);
		move_base.publish(new Vector3(lx,ly,lz));
	}

	public void keep_moving(int period, double lx, double ly, double lz, double ax, double ay, double az) {
		PeriodicPublisher cmd_vel = new PeriodicPublisher("/cmd_vel", "geometry_msgs/Twist", bridge);

		Vector3 linear = new Vector3(lx,ly,lz);
		Vector3 angular = new Vector3(ax,ay,az);
		cmd_vel.beginPublishing(new Twist(linear, angular), 2000);
	}

	public void stop_moving() {
		Publisher cmd_vel = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);

		Vector3 linear = new Vector3(0.0,0.0,0.0);
		Vector3 angular = new Vector3(0.0,0.0,0.0);
		cmd_vel.publish(new Twist(linear, angular));
	}

	public void control_arm(String modereq) {
		Publisher arm_control = new Publisher("/gwendolen_curiosity_arm", "std_msgs/String", bridge);
		arm_control.publish(new PrimitiveMsg<String>(modereq));
	}

	public void control_mast(String modereq) {
		Publisher mast_control = new Publisher("/gwendolen_curiosity_mast", "std_msgs/String", bridge);
		mast_control.publish(new PrimitiveMsg<String>(modereq));
	}

	public void control_wheels(String modereq, int distancereq) {
		Publisher wheels_control = new Publisher("/gwendolen_curiosity_wheels", "std_msgs/String", bridge);
		Publisher wheels_distance = new Publisher("/gwendolen_curiosity_wheels_distance", "std_msgs/Int64", bridge);
		wheels_control.publish(new PrimitiveMsg<String>(modereq));
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		wheels_distance.publish(new PrimitiveMsg<Integer>(distancereq));
	}

	@Override
	public boolean done() {
		return false;
	}

}
