import time
from header import *
from log_parking import *
from recurring_timer import *
from packet_queue import *


CODE_VNC = "VNC"
SENDING = 1		#queue not empty
NOSENDING = 0		#queue empty


#Timer for sending buffer.
#	protected:
#		virtual void expire(Event *e);
#		VNCAgent *a_;;
#The sending timer times out, check the buffer status.
#void SendingTimer::expire(Event *e)
#	a_.check_buffer_status();

class VNCAgent(object):
	# Agent/VNCAgent constructor 
	def __init__(self, id, max_node_id_, sn_size_, num_srcs_, clock_msg_enabled_):
		# TODO binds
		#bind_offset(&hdr_vns::offset_);
#		self.port_number_ =
#		self.server_port_ =
		self.nodeID_ = id
#		self.send_wait_ =
#		self.packetSize_ =
#		self.client_state_ =
#		self.leader_status_ =
		# end binds

#		srand(time(NULL)+nodeID_);

		self.queue = CustomQueue() #queue = (struct PacketQ *) malloc(sizeof(struct PacketQ *)); #queue.init();

		self.sending_status_ = NOSENDING;
		
		# TODO set initial region
		self.regionX_ = -1;
		self.regionY_ = -1;

		#TODO shmid log stuff
		
		self.app_code = UNKNOWN;


	# The method processing every packet received by the agent
	def recv(self, pkt):
		# Access the Ping header for the received packet:
		#hdr_vns * hdr = hdr_vns::access(pkt);
		vnhdr = pkt.vnhdr

		if(vnhdr.type == LEADER_MSG):
			if(vnhdr.subtype == NEWLEADER):
				#log_info(CODE_ADDRC, RECV_INTERNAL, "Agent/VNC: leader infomation received.");
				#The local server is the leader.
				self.leader_status_ = LEADER;
			elif(vnhdr.subtype == NONLEADER):
				#log_info(CODE_ADDRC, RECV_INTERNAL, "Agent/VNC: leader infomation received.");
				#the local server is non the leader
				self.leader_status_ = NON_LEADER;
		elif(vnhdr.type == REGION_MSG):
			if(vnhdr.subtype == NEWREGION):
				self.regionX_ = vnhdr.regionX;#update the region id anyways
				self.regionY_ = vnhdr.regionY;

				if(self.client_state_ == UNKNOWN):#Do this only when a client node is uninitialized
					self.init();
				else:
					self.reset();#reset something when later the node enters a new region.
		elif(vnhdr.type == SYNC_MSG):
			pass #do nothing for SYN messages
		#elif(hdr.type == SYN_REQUEST || hdr.type == SYN_ACK):
			#pass #do nothing for SYN messages
		elif(vnhdr.type == APPL_MSG):#unknown message type
			if(self.client_state_ != UNKNOWN):
				#if(LOG_ENABLED):
				#  log_info(CODE_VNC, RECV_APPL, "Agent/VNC.");
				self.handle_packet(pkt);
				self.send_packets();
			else:
				pass
				#the node is not supposed to process any packet if it's status is unknown
				#dead node

#		Packet::free(pkt);
		#log_info(CODE_VNC, PFREE, "packet freed");
		return
	
	
	#the packet handler of the application client, to be overriden by subclasses.
	def handle_packet(self, p):
		return
		
		
	#initialize a client application. to be overriden by subclasses.
	def init(self, ):
		return
		
		
	#reset a client application's state each time the node enters a region, to be overriden by subclasses.
	def reset(self, ):
		return
		
		
	#send the messages in the message buffer, if there is any
	def send_packets(self, ):
		#log_info(CODE_VNC, QUEUE, (double)sending_status_);
		if(self.sending_status_ == SENDING):
			#just wait for the next timeout
			pass #log_info(CODE_VNS, QUEUE, "sending ongoing, wait");
		else:
			if(self.queue.size() != 0):
				self.sending_status_ = SENDING;
				#log_info(CODE_VNS, QUEUE, "sending enabled for the queue");
				self.send_timer_.resched(self.send_wait_);
			else:
				pass #log_info(CODE_VNS, QUEUE, "empty");
				
				
	# Awaken by the client timer.
	# Check the current buffer status to see if anything needs to be done 
	def check_buffer_status(self, ):
		p = None # Packet * p;
		
		if(self.queue.size() !=0):
			self.sending_status_ = SENDING;
			p = self.queue.pop();
			if(p):
				hdr = p.vnhdr
				if(hdr.send_type == SEND):
					if(LOG_ENABLED):
						self.log(self.app_code,SEND,self.getAppHeader(p))
				else:
					pass
				send(p,0)
			self.send_timer_.resched(self.send_wait_);
		else:
			#log_info(CODE_ADDRC,QUEUE,"Queue emptied");
			self.sending_status_ = NOSENDING;


	#get application packet header into a string
	def getAppHeader(self, pkt):
		return "" #the memory used need to be released by the user of this method.
