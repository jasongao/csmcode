package edu.mit.csail.jasongao.vnconsistent;

import java.io.Serializable;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

public class CSMLayer implements Serializable {
	private static final long serialVersionUID = 5L;

	// Attributes
	public boolean active = false;
	public boolean cacheEnabled;
	public boolean synced;
	public boolean forwardingEnabled;
	RegionKey region;
	public Map<RegionKey, Block> blocks;

	public class Block implements Serializable {
		private static final long serialVersionUID = 5L;

		public Map<String, byte[]> lines;

		Block() {
			this.lines = new ConcurrentHashMap<String, byte[]>();
		}

		Block(Block other) {
			this.lines = new ConcurrentHashMap<String, byte[]>(other.lines);
		}
	}

	// Prevent forwarding loops
	// map source region to an incrementing long
	private long nextOpNonce = 0;
	transient private Map<RegionKey, Set<Long>> forwardedPackets;

	// References to other components
	// Do not serialize these
	transient private Mux mux;
	transient private VNCDaemon vncDaemon;
	transient public UserServer userServer;

	transient private Runnable pendingRequestsRetryCheckR;

	private static final long pendingRequestsRetryCheckPeriod = 600;
	private static final long pendingRequestsRetryTimeoutPeriod = 700;

	// CSM procedure retry and at-most-once queues
	// Requester side
	private Map<RegionKey, Long> nextRequestId;
	private Map<RegionKey, Map<Long, CSMOp>> pendingRequests;
	// Procedure servicer side
	private Map<RegionKey, Map<Long, CSMOp>> sentReplies;

	// INSO
	// Write updater / HOME side
	private long globalOrder; // my monotonically increasing write update order
	private Map<Long, CSMOp> sentWriteUpdates; // Store for retries
	private Map<Long, Set<RegionKey>> acksNotReceived; // who hasn't ack'd?
	// Acknowledger / other side
	private Map<RegionKey, Long> remoteOrders;
	private Map<RegionKey, Map<Long, CSMOp>> writeBuffer;

	/** Enable CSM cache */
	public synchronized void enableCaching() {
		logMsg("*** CSM Caching Enabled ***");
		this.cacheEnabled = true;
	}

	/** Disable CSM cache */
	public synchronized void disableCaching() {
		logMsg("*** CSM Caching Disabled ***");
		this.cacheEnabled = false;
	}

	/** CSM-UserServer Interface - make a procedure call request */
	public synchronized long procedureCallRequest(int p, long rx, long ry,
			boolean write) {
		if (!active)
			return -1L;

		// Ensure request is uniquely identified by region-region pair and an id
		RegionKey dstRegion = new RegionKey(rx, ry);
		Long id = nextRequestId.get(dstRegion);

		if (id == null) {
			id = 0L;
		}
		nextRequestId.put(dstRegion, id + 1);
		CSMOp request = new CSMOp(id, p, CSMOp.PROC_REQUEST, region,
				new RegionKey(rx, ry));
		request.isWrite = write;

		// Include most recent completed request, so remote HOME can cull logs
		request.lowestPendingRequestId = this.lowestPendingRequestId(dstRegion);
		if (request.lowestPendingRequestId < 0)
			request.lowestPendingRequestId = request.requestId;

		logMsg(String.format("Sending %s", request));
		pendingRequests.get(request.dstRegion).put(request.requestId, request);

		// handle cached read-only procedure requests
		if (this.cacheEnabled && !request.dstRegion.equals(this.region)
				&& request.isWrite == false
				&& request.type == CSMOp.PROC_REQUEST
				&& this.blocks.get(request.dstRegion) != null) {
			logMsg(String.format("Local cache hit on read-only %s", request));
			CSMOp reply = userServer.handleCSMRequest(
					this.blocks.get(request.dstRegion), request);
			this.dispatchCSMOp(reply);
		} else {
			this.dispatchCSMOp(request);
		}
		return id;
	}

	/**
	 * This function receives CSMOps destined for remote and local regions. It
	 * ensures at-most-once execution by filtering out repeated CSMOps and
	 * orders operations for INSO.
	 */
	public synchronized void handleCSMOp(final CSMOp op) {
		if (!active || userServer == null) {
			return; // don't do anything until fully started
		}

		// handle writes and non-cacheable procedures or non-cached procedures
		if (op.dstRegion.equals(this.region) || op.broadcast) {
			switch (op.type) {
			case CSMOp.PROC_REQUEST:
				this.cullSentReplies(op.srcRegion, op.lowestPendingRequestId);

				if (sentReplies.get(op.srcRegion).containsKey(op.requestId)) { // at-most-once
					CSMOp reply = sentReplies.get(op.srcRegion).get(
							op.requestId);

					logMsg(String.format("Received DUPLICATE %s, replying %s",
							op, reply));

					this.dispatchCSMOp(reply);
					return;
				} else {
					if (!this.blocks.containsKey(this.region)) {
						this.blocks.put(this.region, new Block());
					}
					CSMOp reply = userServer.handleCSMRequest(
							this.blocks.get(region), op);
					logMsg(String.format("Received %s, replying %s", op, reply));
					sentReplies.get(reply.dstRegion)
							.put(reply.requestId, reply);
					this.dispatchCSMOp(reply);

					if (this.cacheEnabled && op.isWrite) {
						logMsg("Sending out write updates with threadgroup");
						// send out WRITE_UPDATE with broadcast flag
						CSMOp update = new CSMOp(-1, -1, CSMOp.WRITE_UPDATE,
								region, new RegionKey(-2, -2));
						update.broadcast = true;
						update.block = new Block(blocks.get(region));
						update.order = globalOrder;
						globalOrder++;

						// Save for later WRITE_UPDATE_REQUESTS / retries
						sentWriteUpdates.put(update.order, update);
						Set<RegionKey> allOtherRegions = new HashSet<RegionKey>();
						for (long x = 0; x <= vncDaemon.maxRx; x++) {
							for (long y = 0; y <= vncDaemon.maxRy; y++) {
								allOtherRegions.add(new RegionKey(x, y));
							}
						}
						// remove ourself
						allOtherRegions.remove(region);
						acksNotReceived.put(update.order, allOtherRegions);
						dispatchCSMOp(update);
						/*
						 * writeUpdateThread = new Thread( new
						 * ThreadGroup("writeUpdateGroup").getParent(),
						 * this.writeUpdateR, "writeUpdateThread", 256L);
						 * writeUpdateThread.start();
						 */
					}
				}
				break;
			case CSMOp.PROC_REPLY: // pass to userserver
				// ignore if not in pending request queue, duplicate reply
				if (pendingRequests.get(op.srcRegion).containsKey(op.requestId)) {
					logMsg(String.format("Received %s, handing to UserServer",
							op));
					pendingRequests.get(op.srcRegion).remove(op.requestId);
					userServer.handleCSMReply(op);
				} else {
					logMsg(String.format("Received DUPLICATE %s", op));
				}
				break;
			case CSMOp.WRITE_UPDATE: // update cache and send back an ack
				// don't apply our own updates, though
				logMsg(String.format("Received %s", op));
				if (this.cacheEnabled && !op.srcRegion.equals(this.region)) {
					if (!this.remoteOrders.containsKey(op.srcRegion)) {
						// first write_update heard from this remote region
						logMsg(String.format(
								"first WRITE_UPDATE from %s is order %d",
								op.srcRegion, op.order));
						this.remoteOrders.put(op.srcRegion, op.order);
					}

					// ignore if < next order expected; odd stack overflow err
					long nextOrder = this.remoteOrders.get(op.srcRegion);
					if (op.order >= nextOrder) {
						this.writeBuffer.get(op.srcRegion).put(op.order, op);

						// send back ack
						CSMOp ack = new CSMOp(-1, -1, CSMOp.WRITE_UPDATE_ACK,
								this.region, op.srcRegion);
						ack.order = op.order;
						this.dispatchCSMOp(ack);

						// try to apply write updates
						checkWriteBuffer(op.srcRegion);
					}
				}
				break;
			case CSMOp.WRITE_UPDATE_ACK:
				if (this.cacheEnabled) {
					logMsg(String.format("Received %s", op));
					// remove op.srcRegion from List of waiting-for-ack
					if (this.acksNotReceived.containsKey(op.order)) {
						this.acksNotReceived.get(op.order).remove(op.srcRegion);
						if (this.acksNotReceived.get(op.order).size() <= 0) {
							// if waiting-for-ack List is empty, it's complete,
							// so remove from List of pending updates
							logMsg("WRITE_UPDATE:" + op.order
									+ " completed on all other regions.");
							this.acksNotReceived.remove(op.order);
							this.sentWriteUpdates.remove(op.order);
						}
					}
				}
				break;
			case CSMOp.WRITE_UPDATE_REQUEST: // resend the write_update
				if (this.cacheEnabled) {
					logMsg(String.format("Received %s", op));
					CSMOp updateToResend = this.sentWriteUpdates.get(op.order);
					if (updateToResend != null) {
						logMsg(String.format("Sending requested %s", op));
						this.dispatchCSMOp(updateToResend);
					} else {
						logMsg(String
								.format("Can't resend requested update, not found in sentWriteupdates"));
					}
				}
				break;
			}
		} else if (forwardingEnabled) {
			if (!this.forwardedPackets.containsKey(op.srcRegion)) {
				this.forwardedPackets.put(new RegionKey(op.srcRegion),
						new HashSet<Long>());
			}

			// Don't forward an op more than once
			if (!this.forwardedPackets.get(op.srcRegion).contains(op.nonce)) {
				// Forward on towards destination, x-y routing
				if (op.dstRegion.x == region.x && op.srcRegion.y < region.y
						&& region.y < op.dstRegion.y) {
					logMsg("Forwarding CSMOp to remote region.");
					this.forwardedPackets.get(op.srcRegion).add(op.nonce);
					this.dispatchCSMOp(op);
				} else if (op.srcRegion.x < region.x
						&& region.x < op.dstRegion.x) {
					logMsg("Forwarding CSMOp to remote region.");
					this.forwardedPackets.get(op.srcRegion).add(op.nonce);
					this.dispatchCSMOp(op);
				} else if (op.broadcast) {
					logMsg("Forwarding CSMOp because it's broadcast.");
					this.forwardedPackets.get(op.srcRegion).add(op.nonce);
					this.dispatchCSMOp(op);
				}
			} else {
				logMsg("Received CSMOp already forwarded, ignoring...");
			}
		}
	}

	/** go through buffered write updates and apply the ones we can */
	private void checkWriteBuffer(RegionKey r) {
		long nextOrder = this.remoteOrders.get(r);

		logMsg(String
				.format("Applying write updates from %s, remote order = %d, write buffer contains %s",
						r, nextOrder, this.writeBuffer.get(r).keySet()));

		if (this.writeBuffer.get(r).containsKey(nextOrder)) {
			// apply next update
			CSMOp nextUpdate = this.writeBuffer.get(r).get(nextOrder);
			this.blocks.put(r, new Block(nextUpdate.block));
			this.writeBuffer.get(r).remove(nextOrder);

			// and increment remote order expected
			this.remoteOrders.put(r, ++nextOrder);

			logMsg(String.format("Applied WRITE_UPDATE:%d from %s",
					nextUpdate.order, r));
			checkWriteBuffer(r); // recursively check next expected order
		} else if (this.writeBuffer.get(r).size() > 0) {
			// doesn't contain next order, but other updates are waiting
			// so request the missing order
			logMsg(String.format("Missing WRITE_UPDATE:%d from %s, requesting",
					nextOrder, r));
			CSMOp retryRequest = new CSMOp(-1, -1, CSMOp.WRITE_UPDATE_REQUEST,
					this.region, r);
			retryRequest.order = nextOrder;
			this.dispatchCSMOp(retryRequest);
		}
	}

	private void cullSentReplies(RegionKey rk, long lowestPendingRequestId) {
		Map<Long, CSMOp> regionSentReplies = this.sentReplies.get(rk);
		Iterator<Entry<Long, CSMOp>> it = regionSentReplies.entrySet()
				.iterator();
		while (it.hasNext()) {
			Map.Entry<Long, CSMOp> pairs = (Map.Entry<Long, CSMOp>) it.next();
			long id = pairs.getValue().requestId;
			if (id < lowestPendingRequestId)
				it.remove();
		}
		logMsg("removed replies before id " + lowestPendingRequestId
				+ " from sentReplies of size " + sentReplies.get(rk).size());
	}

	private long lowestPendingRequestId(RegionKey rk) {
		long lowestId = -1;
		Map<Long, CSMOp> regionPendingRequests = this.pendingRequests.get(rk);
		for (long id : regionPendingRequests.keySet()) {
			if (lowestId == -1 || id < lowestId)
				lowestId = id;
		}
		return lowestId;
	}

	/** CSMLayer constructor */
	public CSMLayer(VNCDaemon vnc, RegionKey r, boolean cachen) {
		this.vncDaemon = vnc;
		this.cacheEnabled = cachen;
		this.forwardingEnabled = true;
		this.synced = false;

		logMsg("*** Starting CSM Layer ***");
		if (this.cacheEnabled) {
			logMsg("*** CSM Layer starting with cache enabled ***");
		} else {
			logMsg("*** CSM Layer starting with cache disabled ***");
		}
		if (this.forwardingEnabled) {
			logMsg("*** CSM Layer starting with forwarding enabled ***");
		} else {
			logMsg("*** CSM Layer starting with forwarding disabled ***");
		}

		this.region = new RegionKey(r);

		this.blocks = new ConcurrentHashMap<RegionKey, Block>();
		for (long x = 0; x <= this.vncDaemon.maxRx; x++) {
			for (long y = 0; y <= this.vncDaemon.maxRy; y++) {
				this.blocks.put(new RegionKey(x, y), new Block());
			}
		}

		this.nextRequestId = new ConcurrentHashMap<RegionKey, Long>();

		this.pendingRequests = new ConcurrentHashMap<RegionKey, Map<Long, CSMOp>>();
		for (long x = 0; x <= this.vncDaemon.maxRx; x++) {
			for (long y = 0; y <= this.vncDaemon.maxRy; y++) {
				this.pendingRequests.put(new RegionKey(x, y),
						new ConcurrentHashMap<Long, CSMOp>());
			}
		}

		this.sentReplies = new ConcurrentHashMap<RegionKey, Map<Long, CSMOp>>();
		for (long x = 0; x <= this.vncDaemon.maxRx; x++) {
			for (long y = 0; y <= this.vncDaemon.maxRy; y++) {
				this.sentReplies.put(new RegionKey(x, y),
						new ConcurrentHashMap<Long, CSMOp>());
			}
		}

		// INSO / SRCC
		this.writeBuffer = new ConcurrentHashMap<RegionKey, Map<Long, CSMOp>>();
		for (long x = 0; x <= this.vncDaemon.maxRx; x++) {
			for (long y = 0; y <= this.vncDaemon.maxRy; y++) {
				this.writeBuffer.put(new RegionKey(x, y),
						new ConcurrentHashMap<Long, CSMOp>());
			}
		}

		this.sentWriteUpdates = new ConcurrentHashMap<Long, CSMOp>();
		this.acksNotReceived = new ConcurrentHashMap<Long, Set<RegionKey>>();
		this.remoteOrders = new ConcurrentHashMap<RegionKey, Long>();
	}

	public synchronized void start(Mux m, VNCDaemon v) {
		this.mux = m;
		this.vncDaemon = v;

		logMsg("CSMLayer starting");
		this.userServer = new UserServer(m, this);
		this.userServer.start();

		this.active = true;

		this.forwardedPackets = new HashMap<RegionKey, Set<Long>>();
		for (long x = 0; x <= this.vncDaemon.maxRx; x++) {
			for (long y = 0; y <= this.vncDaemon.maxRy; y++) {
				this.forwardedPackets.put(new RegionKey(x, y),
						new HashSet<Long>());
			}
		}

		// Setup timeout watchdog timer
		pendingRequestsRetryCheckR = new Runnable() {
			@Override
			public void run() {
				for (RegionKey rk : pendingRequests.keySet()) {
					Map<Long, CSMOp> rPendingRequests = pendingRequests.get(rk);

					Iterator<Entry<Long, CSMOp>> it = rPendingRequests
							.entrySet().iterator();
					while (it.hasNext()) {
						Map.Entry<Long, CSMOp> pairs = (Map.Entry<Long, CSMOp>) it
								.next();
						CSMOp request = pairs.getValue();
						long timeSinceRequest = System.currentTimeMillis()
								- request.timestamp;

						if (timeSinceRequest > 3 * pendingRequestsRetryTimeoutPeriod) {
							// it.remove(); // remove in reply handling instead
							CSMOp reply = new CSMOp(request.requestId,
									request.procedure, CSMOp.PROC_REPLY,
									request.dstRegion, request.srcRegion);
							reply.timedOut = true;
							logMsg("Request timed out, send failure " + reply);
							sentReplies.get(reply.dstRegion).put(
									reply.requestId, reply);
							dispatchCSMOp(reply);
						} else if (timeSinceRequest > pendingRequestsRetryTimeoutPeriod) {
							logMsg("Retrying " + request);
							dispatchCSMOp(request);
						}
					}
				}
				vncDaemon.myHandler.postDelayed(this,
						pendingRequestsRetryCheckPeriod);
			}
		};
		vncDaemon.myHandler.postDelayed(pendingRequestsRetryCheckR,
				pendingRequestsRetryCheckPeriod);
	}

	public synchronized void init() {
		this.userServer.init();
	}

	public synchronized void stop() {
		this.active = false;
		vncDaemon.myHandler.removeCallbacks(pendingRequestsRetryCheckR);
		if (this.userServer != null)
			this.userServer.stop();
		this.userServer = null;
		logMsg("CSMLayer stopped");
		return;
	}

	/** Log message to device display and to Android log. */
	private void logMsg(String msg) {
		vncDaemon.logMsg(msg);
	}

	/** Send a CSMOp to the Mux, which will loopback or broadcast as necessary */
	private void dispatchCSMOp(CSMOp op) {
		logMsg("Dispatching CSMOp " + op);
		op.nonce = nextOpNonce++;
		mux.myHandler.obtainMessage(Mux.CSM_SEND, op).sendToTarget();
	}
}