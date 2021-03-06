package edu.mit.csail.jasongao.vnconsistent;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import android.util.Log;

public class UserServer implements CSMUser {
	final static String TAG = "UserServer";

	private CSMLayer csm;
	private Mux mux;

	// CSM Procedures that can be called
	final static int INIT = 0;
	final static int INCREMENT = 1;
	final static int DECREMENT = 2;
	final static int READ = 3;

	// Map Region, CSMRequestID, ClientID
	private Map<RegionKey, Map<Long, Long>> clientRequests;

	/** Log message to device display and to Android log. */
	public void logMsg(String line) {
		line = String.format("%d: %s", System.currentTimeMillis(), line);
		mux.myHandler.obtainMessage(Mux.LOG, line).sendToTarget();
		Log.i(TAG, line);
	}

	/** UserServer constructor */
	public UserServer(Mux m, CSMLayer c) {
		this.mux = m;
		this.csm = c;
		clientRequests = new ConcurrentHashMap<RegionKey, Map<Long, Long>>();
	}

	/** Called only upon empty CSMLayer invocation in region. */
	public void init() {
		logMsg("UserServer initialized.");
		long x = csm.region.x, y = csm.region.y;
		logMsg(String.format("Requesting INIT on %d,%d", x, y));
		long requestId = csm.procedureCallRequest(INIT, x, y, true);
	}

	/** Called every time upon starting / resuming userServer in a region */
	public void start() {
		logMsg("UserServer started.");
	}

	/** Called upon stopping userServer in a region, e.g. switch leader */
	public void stop() {
		logMsg("UserServer stopped.");
	}

	/** Handle an application-specific packet received through the network */
	public void handleClientRequest(UserOp uop) {
		logMsg(String.format("UserServer handling request in (%d,%d)",
				uop.requesterRx, uop.requesterRy));

		long requestId = -1;

		switch (uop.type) {
		case UserOp.INCREMENT:
			requestId = csm.procedureCallRequest(INCREMENT, uop.targetRx,
					uop.targetRy, true);
			break;
		case UserOp.DECREMENT:
			requestId = csm.procedureCallRequest(DECREMENT, uop.targetRx,
					uop.targetRy, true);
			break;
		case UserOp.READ:
			requestId = csm.procedureCallRequest(READ, uop.targetRx,
					uop.targetRy, false);
			break;
		case UserOp.INIT:
			requestId = csm.procedureCallRequest(INIT, uop.targetRx,
					uop.targetRy, true);
			break;
		}

		// store request id+region to client id mapping
		RegionKey targetRegion = new RegionKey(uop.targetRx, uop.targetRy);
		if (!clientRequests.containsKey(targetRegion))
			clientRequests.put(targetRegion,
					new ConcurrentHashMap<Long, Long>());
		clientRequests.get(targetRegion).put(requestId, uop.requesterId);

		return;
	}

	/**
	 * Handle a CSM procedure call reply from a remote region. Replies should
	 * only be received once; use of nonce to filter out beforehand
	 */
	public synchronized void handleCSMReply(CSMOp reply) {
		// lookup the corresponding client that generated this request
		long clientId = -1;
		if (reply.procedure != INIT) {
			Map<Long, Long> idToClientMap = clientRequests.get(reply.srcRegion);
			if (idToClientMap != null
					&& idToClientMap.containsKey(reply.requestId)) {
				clientId = idToClientMap.get(reply.requestId);
				idToClientMap.remove(reply.requestId);
			}
		}

		if (reply.timedOut) {
			logMsg(String.format("Procedure %d:%d on %s timed out, sending failure reply to client", reply.procedure,
					reply.requestId, reply.srcRegion));

			// reply back to client that their request failed
			if (reply.procedure != this.INIT) {
				UserOp uopReply = new UserOp(clientId, reply.procedure,
						csm.region.x, csm.region.y, reply.srcRegion.x,
						reply.srcRegion.y);
				uopReply.request = false;
				uopReply.success = false;
				mux.myHandler.obtainMessage(Mux.APP_SEND, uopReply)
						.sendToTarget();
			}

		} else {
			logMsg(String.format("Procedure %d:%d on %s successful", reply.procedure,
					reply.requestId, reply.srcRegion));

			long val = -1337;
			try {
				ByteArrayInputStream bis = new ByteArrayInputStream(reply.data);
				DataInputStream dis = new DataInputStream(bis);
				val = dis.readLong();
				dis.close();
				bis.close();
			} catch (Exception e) {
				logMsg("User proc reply data parse error: " + e.getMessage());
			}

			// reply back to client with the data they requested.
			if (reply.procedure != this.INIT) {
				UserOp uopReply = new UserOp(clientId, reply.procedure,
						csm.region.x, csm.region.y, reply.srcRegion.x,
						reply.srcRegion.y);
				uopReply.request = false;
				uopReply.value = val;
				mux.myHandler.obtainMessage(Mux.APP_SEND, uopReply)
						.sendToTarget();
			}
		}
	}

	/** Handle and reply to a CSM procedure call request on the local region. */
	public synchronized CSMOp handleCSMRequest(CSMLayer.Block block,
			final CSMOp request) {
		CSMOp reply = new CSMOp(request.requestId, request.procedure,
				CSMOp.PROC_REPLY, request.dstRegion, request.srcRegion);
		switch (request.procedure) {
		case READ:
			reply.data = block.lines.get("the_value");
			reply.requestSuccess = true;
			break;
		case INIT:
			long initVal = 20L;
			this.writeLongValue(block, initVal);
			reply.data = block.lines.get("the_value");
			reply.requestSuccess = true;
			break;
		case INCREMENT:
			long incVal = this.readLongValue(block);
			incVal++;
			this.writeLongValue(block, incVal);
			reply.data = block.lines.get("the_value");
			reply.requestSuccess = true;
			break;
		case DECREMENT:
			long decVal = this.readLongValue(block);
			decVal--;
			this.writeLongValue(block, decVal);
			reply.data = block.lines.get("the_value");
			reply.requestSuccess = true;
			break;
		}

		return reply;
	}

	public long readLongValue(CSMLayer.Block block) {
		long val = -1337;
		try {
			byte[] data = block.lines.get("the_value");
			ByteArrayInputStream bis = new ByteArrayInputStream(data);
			DataInputStream dis = new DataInputStream(bis);
			val = dis.readLong();
			dis.close();
			bis.close();
		} catch (IOException e) {
			Log.e(TAG, "error callback reading long value: " + e.getMessage());
		}
		return val;
	}

	/** Write our long value to CSM */
	public long writeLongValue(CSMLayer.Block block, long val) {
		try {
			ByteArrayOutputStream bos = new ByteArrayOutputStream();
			DataOutputStream dos = new DataOutputStream(bos);
			dos.writeLong(val);
			dos.close();
			byte[] output = bos.toByteArray();
			bos.close();
			block.lines.put("the_value", output);
		} catch (IOException e) {
			Log.e(TAG, "error writing long value: " + e.getMessage());
		}
		return val;
	}
}