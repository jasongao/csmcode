package edu.mit.csail.jasongao.vnconsistent;

import java.io.Serializable;

public class UserOp implements Serializable {
	private static final long serialVersionUID = 1L;

	// Application message types from client
	// note: does not have to map to CSM procedures, but this is a simple
	// benchmarking app, so it happens to do so here
	final static int INIT = 0;
	final static int INCREMENT = 1;
	final static int DECREMENT = 2;
	final static int READ = 3;

	// attributes to be serialized
	public int type;
	public long targetRx, targetRy, requesterRx, requesterRy;
	public long requesterId;
	public long value = -1;
	public boolean request; // if false, then it's a reply
	public boolean success;

	public UserOp(long rId, int type, long srcRx, long srcRy, long dstRx, long dstRy) {
		this.type = type;
		this.targetRx = dstRx;
		this.targetRy = dstRy;
		this.requesterRx = srcRx;
		this.requesterRy = srcRy;
		this.request = true;
		this.success = true;
		this.requesterId = rId;
	}
}
