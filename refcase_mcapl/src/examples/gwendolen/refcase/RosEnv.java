package gwendolen.refcase;
import ail.mas.DefaultEnvironment;
import ail.syntax.*;
import com.fasterxml.jackson.databind.JsonNode;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;

//for metrics
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

public class RosEnv extends DefaultEnvironment {

	private static final String ROS_URL = "ws://localhost:9090";

	private static final String CONTROL_TOPIC = "/gwendolen_control";
	private static final String CONTROL_TYPE = "std_msgs/Bool";
	
	private static final String STATE_TOPIC = "/gwendolen_state";
	private static final String STATE_TYPE = "std_msgs/String";

	private static final String STATETIME_TOPIC = "/gwendolen_state_with_time";
	private static final String STATETIME_TYPE = "std_msgs/String";

	private static final String MOVE_TOPIC = "/gwendolen_move_percept";
	private static final String MOVE_TYPE = "std_msgs/Bool";

	private static final String TICK_TOPIC = "/gwendolen_tick";
	private static final String TICK_TYPE = "std_msgs/Bool";

	private static final String GEOFENCEVIOLATION_TOPIC = "/gwendolen_geofence_violation";
	private static final String GEOFENCEVIOLATION_TYPE = "std_msgs/Bool";

	private static final String GVTIME_TOPIC = "/gwendolen_geofence_violation_time";
	private static final String GVTIME_TYPE = "std_msgs/String";
	
	private static final String SCAN_TOPIC = "/scan";
	private static final String SCAN_TYPE = "sensor_msgs/LaserScan";
	private static final double SAFE_DISTANCE_THRESHOLD = 0.20;
	
	// AgileX Scout Mini physical dimensions (in meters)
	private static final double ROBOT_LENGTH = 0.612;  // Length (front to back)
	private static final double ROBOT_WIDTH = 0.580;   // Width (left to right)

	private static final String HALT_TOPIC = "/safehalt_request";
	private static final String HALT_TYPE = "std_msgs/Bool";

	private static final String ODOM_TOPIC = "/odom";
	private static final String ODOM_TYPE ="nav_msgs/Odometry";
	private boolean lastStoppedState = false;
	private boolean geofenceViolationActive = false;
	private boolean safeHaltRequestActive = false;

	private final RosBridge bridge;

	//for metrics

	private final String runId =
        new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss")
                .format(new Date());

	private boolean enforcedStopOccurred = false;
	private boolean safeHaltActivated = false;
	private boolean geofenceViolationOccurred = false;
	private double minDistanceObserved = Double.MAX_VALUE;
	private Long enforcedStopTimestamp = null;
	private Long safeHaltTimestamp = null;
	private Long geofenceViolationTimestamp = null;

	// State & event counters (for per-run metrics and later aggregation)
	private String currentState = "Monitoring";
	private long stateEnteredTimestamp = System.currentTimeMillis();

	private long monitoringTimeMs = 0;
	private long waitForHaltTimeMs = 0;
	private long safeHaltActiveTimeMs = 0;

	private long geofenceCountMonitoring = 0;
	private long geofenceCountWaitForHalt = 0;
	private long geofenceCountSafeHaltActive = 0;

	private long haltObservedInWaitCount = 0;
	private long tickInWaitCount = 0;

	private long waitForHaltEntryCount = 0;
	private long safeHaltActiveEntryCount = 0;

	private long moveInSafeHaltCount = 0;
	private long safeHaltRequestCount = 0;
	private Long safeHaltRequestTimestamp = null;

	private static final String LOG_FILE = "ss_metrics.csv";

	
	public RosEnv() {
		super();
		bridge = new RosBridge();
		bridge.connect(ROS_URL, true);
		System.out.println("Environment started, connection with ROS established.");
		
		publishState("Monitoring");
		System.out.println("Publishing state Monitoring");

		// Subscribe to laser scan topic
		bridge.subscribe(
				SubscriptionRequestMsg.generate(SCAN_TOPIC)
						.setType(SCAN_TYPE),
				new RosListenDelegate() {
					@Override
					public void receive(JsonNode data, String stringRep) {
						handleLaserScanData(data);
					}
				});

		// Subscribe to safeHaltRequested topic
		bridge.subscribe(
				SubscriptionRequestMsg.generate(HALT_TOPIC).setType(HALT_TYPE),
				new RosListenDelegate() {
					@Override
					public void receive(JsonNode msg, String s) { handleSafeHaltRequested(msg);}
				}
		);

		// Subcribe to odometry Topic -> To check if the robot is still moving
		bridge.subscribe(
				SubscriptionRequestMsg.generate(ODOM_TOPIC)
						.setType(ODOM_TYPE),
				new RosListenDelegate() {
					@Override
					public void receive(JsonNode msg, String s) { handleOdometryData(msg); }
				}
		);

		// Register shutdown hook
		Runtime.getRuntime().addShutdownHook(
				new Thread(() -> {
					System.out.println("Writing run metrics...");
					writeRunMetrics();
				}));

	}


	private void handleOdometryData(JsonNode msg){

		if (msg == null) return;

		JsonNode twistNode = msg.path("msg").path("twist").path("twist");
		//System.out.println("RAW ODOM" + twistNode.toString());

		if (twistNode.isMissingNode()) {
			return; // Not a valid odometry message
		}

		double linearX = twistNode.path("linear").path("x").asDouble(Double.NaN);
		double linearY = twistNode.path("linear").path("y").asDouble(Double.NaN);
		double angularZ = twistNode.path("angular").path("z").asDouble(Double.NaN);

		// System.out.printf("Odometry -> X: %.3f | Y: %.3f | Z: %.3f%n", linearX, linearY, angularZ);

		if (Double.isNaN(linearX) || Double.isNaN(linearY) || Double.isNaN(angularZ)) {
			return; // Ignore malformed message
		}

		boolean stopped =
			Math.abs(linearX) < 0.01 &&
			Math.abs(linearY) < 0.01 &&
			Math.abs(angularZ) < 0.01;

		if (stopped && !lastStoppedState){
			removePercept(new Literal("move"));
			addPercept(new Literal("halt_observed"));
			// update metrics: halt observed while in WaitForHalt
			synchronized (this) {
				if ("WaitingForHalt".equals(currentState)) {
					haltObservedInWaitCount++;
				}
			}
			// System.out.println("Percept generated: Robot Stopped");
		}

		if (!stopped && lastStoppedState){
			removePercept(new Literal("halt_observed"));
			addPercept(new Literal("move"));
			// update metrics: move observed while in SafeHaltActive
			synchronized (this) {
				if ("SafeHaltActive".equals(currentState)) {
					moveInSafeHaltCount++;
				}
			}
			// System.out.println("Percept generated: move");
		}

		lastStoppedState = stopped;

	}


	private void handleSafeHaltRequested(JsonNode msg){

		if (msg == null) return;

		JsonNode dataNode = msg.path("msg").path("data");

		if (dataNode.isMissingNode()){
			return;
		}

		boolean haltRequested = dataNode.asBoolean(false);

		if (haltRequested && !safeHaltRequestActive){
			addPercept(new Literal ("safe_halt_req"));
			synchronized (this) {
				safeHaltRequestCount++;
				safeHaltRequestTimestamp = System.currentTimeMillis();
			}
			System.out.println("Percept generated: safe_halt_req");
			safeHaltRequestActive = true;
		}
	}


	private void handleLaserScanData(JsonNode data) {
		JsonNode msgNode = data.get("msg");
		if (msgNode == null) return;
		
		JsonNode ranges = msgNode.path("ranges");
		double angleMin = msgNode.path("angle_min").asDouble(0.0);
		double angleIncrement = msgNode.path("angle_increment").asDouble(0.0);
		double rangeMin = msgNode.path("range_min").asDouble(0.0);
		double rangeMax = msgNode.path("range_max").asDouble(0.0);

		double minTrueDist = calculateMinTrueDistance(ranges, angleMin, angleIncrement, rangeMin, rangeMax);

		// Track minimum observed distance for metrics
		if (minTrueDist < minDistanceObserved) {
			minDistanceObserved = minTrueDist;
		}

		//System.out.printf("Minimum True Distance: %f%n", minTrueDist);

		if (minTrueDist < SAFE_DISTANCE_THRESHOLD && !geofenceViolationActive) {

			// Update metrics
			geofenceViolationOccurred = true;
			if (geofenceViolationTimestamp == null) {
				geofenceViolationTimestamp = System.currentTimeMillis();
			}

			// increment state-specific geofence counter
			synchronized (this) {
				if ("Monitoring".equals(currentState)) geofenceCountMonitoring++;
				else if ("WaitingForHalt".equals(currentState)) geofenceCountWaitForHalt++;
				else if ("SafeHaltActive".equals(currentState)) geofenceCountSafeHaltActive++;
			}

			publishGeofenceViolation();
			addPercept(new Literal("geofence_violation"));
			geofenceViolationActive = true;
			System.out.println("Percept generated and published: geofence_violation");

		} else if (minTrueDist >= SAFE_DISTANCE_THRESHOLD && geofenceViolationActive) {
			// Distance back to normal, reset
			removePercept(new Literal("geofence_violation"));
			geofenceViolationActive = false;
			System.out.println("Geofence violation cleared");
		}
	}

	private double calculateMinTrueDistance(JsonNode ranges, double angleMin, double angleIncrement, 
										   double rangeMin, double rangeMax) {
		double minTrueDist = Double.MAX_VALUE;

		int i = 0;
		for (JsonNode valueNode : ranges) {
			if (valueNode.isNumber()) {
				double r = valueNode.asDouble();

				// Check if the reading is valid based on sensor limits
				if (rangeMin < r && r < rangeMax && !Double.isInfinite(r)) {

					// 1. Calculate the specific angle (theta) of this ray
					double theta = angleMin + (i * angleIncrement);

					// 2. Prevent division by zero for rays exactly at 0, 90, 180, 270 degrees
					double cosTheta = Math.cos(theta);
					double sinTheta = Math.sin(theta);
					
					if (Math.abs(cosTheta) < 1e-6) {
						cosTheta = 1e-6;
					}
					if (Math.abs(sinTheta) < 1e-6) {
						sinTheta = 1e-6;
					}

					// 3. Calculate the distance from the LiDAR to the robot's edge at this angle
					double rOffset = Math.min(Math.abs((ROBOT_LENGTH / 2.0) / cosTheta), 
									 Math.abs((ROBOT_WIDTH / 2.0) / sinTheta));

					// 4. Subtract the offset to get the distance from the bumper
					double trueDist = r - rOffset;

					// 5. Track the absolute closest point
					if (trueDist < minTrueDist) {
						minTrueDist = trueDist;
					}
				}
			}
			i++;
		}

		// Floor the distance at 0.0 in case an object penetrates the mathematical footprint
		if (minTrueDist == Double.MAX_VALUE) {
			return Double.MAX_VALUE;
		}
		return Math.max(0.0, minTrueDist);
	}


	@Override
	public Unifier executeAction(String agName, Action act) {
		if ("entry_stop".equals(act.getFunctor())) {
			publishStopSignal();
			System.out.println("Sending the message to stop moving");
			publishState("EnforcedStop");
			System.out.println("Publishing state EnforcedStop");

			//update metrics
			enforcedStopOccurred = true;
			if (enforcedStopTimestamp == null) {
				enforcedStopTimestamp = System.currentTimeMillis();
			}
		}

		if ("waiting".equals(act.getFunctor())) {
			new Thread(() -> {
				try {
					Thread.sleep(2000);
					// Record a tick event in WaitForHalt if this is the current state.
					synchronized (this) {
						if ("WaitingForHalt".equals(currentState)) {
							tickInWaitCount++;
						}
					}
					addPercept(new Literal("tick"));
					publishTick();
					System.out.println("Tick percept added and published");
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}).start();
		}

		if ("publish_wait_for_halt".equals(act.getFunctor())) {
			publishState("WaitingForHalt");
			System.out.println("Publishing state WaitingForHalt");
		}

		if ("publish_safe_halt_active".equals(act.getFunctor())) {
			publishState("SafeHaltActive");
			System.out.println("Publishing state SafeHaltActive");

			//update metrics
			safeHaltActivated = true;
			if (safeHaltTimestamp == null) {
				safeHaltTimestamp = System.currentTimeMillis();
			}
		}

		if ("publish_move_percept".equals(act.getFunctor())) {
			publishMovePercept();
			System.out.println("Publishing move percept");
		}

		return new Unifier();
	}

	private void publishStopSignal() {
		Publisher control = new Publisher(CONTROL_TOPIC, CONTROL_TYPE, bridge);
		control.publish(new PrimitiveMsg<>(true));
	}

	private void publishState(String state) {
		long timestamp = System.currentTimeMillis();
		String stateWithTime = state + "@" + timestamp;
		Publisher statePub = new Publisher(STATE_TOPIC, STATE_TYPE, bridge);
		Publisher stateTimePub = new Publisher(STATETIME_TOPIC, STATETIME_TYPE, bridge);
		statePub.publish(new PrimitiveMsg<>(state));
		stateTimePub.publish(new PrimitiveMsg<>(stateWithTime));

		// update internal state tracking, duration totals, and entry counters
		synchronized (this) {
			if (!state.equals(currentState)) {
				long elapsed = timestamp - stateEnteredTimestamp;
				if ("Monitoring".equals(currentState)) {
					monitoringTimeMs += elapsed;
				} else if ("WaitingForHalt".equals(currentState)) {
					waitForHaltTimeMs += elapsed;
				} else if ("SafeHaltActive".equals(currentState)) {
					safeHaltActiveTimeMs += elapsed;
				}
				stateEnteredTimestamp = timestamp;
				this.currentState = state;
				if ("WaitingForHalt".equals(state)) {
					waitForHaltEntryCount++;
				} else if ("SafeHaltActive".equals(state)) {
					safeHaltActiveEntryCount++;
				}
			}
		}
	}

	private void publishMovePercept() {
		Publisher movePub = new Publisher(MOVE_TOPIC, MOVE_TYPE, bridge);
		movePub.publish(new PrimitiveMsg<>(true));
	}

	private void publishTick() {
		Publisher tickPub = new Publisher(TICK_TOPIC, TICK_TYPE, bridge);
		tickPub.publish(new PrimitiveMsg<>(true));
	}

	private void publishGeofenceViolation() {
		long timestamp = System.currentTimeMillis();
		String timeString = "GoefenceViolation@" + timestamp;
		Publisher geofencePub = new Publisher(GEOFENCEVIOLATION_TOPIC, GEOFENCEVIOLATION_TYPE, bridge);
		Publisher gvTimePub = new Publisher(GVTIME_TOPIC, GVTIME_TYPE, bridge);
		geofencePub.publish(new PrimitiveMsg<>(true));
		gvTimePub.publish(new PrimitiveMsg<>(timeString));
	}

	//for metrics
	private void writeRunMetrics() {
		try {
			File file = new File(LOG_FILE);
			boolean newFile = !file.exists();
			try (FileWriter fw = new FileWriter(file, true)) {
				if (newFile) {
						fw.write(
							"run_id," +
							"safe_halt_request," +
							"geofence_violation," +
							"enforced_stop," +
							"safe_halt_active," +
							"min_distance," +
							"safe_halt_request_timestamp," +
							"geofence_violation_timestamp," +
							"enforced_stop_timestamp," +
							"safe_halt_timestamp," +
							"monitoring_time_ms," +
							"waitforhalt_time_ms," +
							"safehaltactive_time_ms," +
							"geofence_count_monitoring," +
							"geofence_count_waitforhalt," +
							"geofence_count_safehaltactive," +
							"haltObserved_in_wait," +
							"tick_in_wait," +
							"waitforhalt_entry_count," +
							"safehaltactive_entry_count," +
							"move_in_safehalt\n"

						);
				}
				long now = System.currentTimeMillis();
				synchronized (this) {
					long elapsed = now - stateEnteredTimestamp;
					if ("Monitoring".equals(currentState)) {
						monitoringTimeMs += elapsed;
					} else if ("WaitingForHalt".equals(currentState)) {
						waitForHaltTimeMs += elapsed;
					} else if ("SafeHaltActive".equals(currentState)) {
						safeHaltActiveTimeMs += elapsed;
					}
					stateEnteredTimestamp = now;
				}
				fw.write(
						runId + "," +
						safeHaltRequestCount + "," +
						(geofenceViolationOccurred ? 1 : 0) + "," +
						(enforcedStopOccurred ? 1 : 0) + "," +
						(safeHaltActivated ? 1 : 0) + "," +
						minDistanceObserved + "," +
						safeHaltRequestTimestamp + "," +
						geofenceViolationTimestamp + "," +
						enforcedStopTimestamp + "," +
						safeHaltTimestamp + "," +
						monitoringTimeMs + "," +
						waitForHaltTimeMs + "," +
						safeHaltActiveTimeMs + "," +
						geofenceCountMonitoring + "," +
						geofenceCountWaitForHalt + "," +
						geofenceCountSafeHaltActive + "," +
						haltObservedInWaitCount + "," +
						tickInWaitCount + "," +
						waitForHaltEntryCount + "," +
						safeHaltActiveEntryCount + "," +
						moveInSafeHaltCount + "\n"
					);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	@Override
	public boolean done() {
		return false;
	}

	
}

