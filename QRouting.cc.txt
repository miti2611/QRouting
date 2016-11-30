#include <agent.h>
#include <packet.h>
#include <trace.h>
#include <timer-handler.h>
#include <random.h>
#include <classifier-port.h>
#include <energy-model.h>
#include <cmath>
#include <address.h>
#include <config.h>
#include <vector>
#include <stdlib.h>
#include <tcl.h>
#include <connector.h>
#include <trace.h>
#include <config.h>
#include <mobilenode.h>

#define CURRENT_TIME Scheduler::instance().clock()
#define JITTER Random::uniform()*0.01

#define QROUTING_ROUTE_DISC 	0x01
#define QROUTING_PKT	   	0x02
#define QROUTING_ACK	   	0x04

#define HDR_QROUTING(p) ((struct hdr_qrouting*)hdr_qrouting::access(p))
#define HDR_ROUTE_DISCOVERY_PKT(p) ((struct hdr_route_discovery_pkt*)hdr_qrouting::access(p))
#define HDR_QROUTING_PKT(p) ((struct hdr_qrouting_pkt*)hdr_qrouting::access(p))
#define HDR_QROUTING_PKT_ACK(p) ((struct hdr_qrouting_pkt_ack*)hdr_qrouting::access(p))

using namespace std;

struct hdr_qrouting
{
public:
	u_int8_t qh_type;


	static int offset_;
	inline static int& offset()
	{
		return (offset_);
	}
	inline static hdr_qrouting* access(const Packet* p)
	{
		return (hdr_qrouting*)p->access(offset_);
	}

};

struct hdr_route_discovery_pkt
{
public:
	u_int8_t rd_type;
	double qenergy_;

};
struct hdr_qrouting_pkt
{
public:
	u_int8_t q_type;
};

struct hdr_qrouting_pkt_ack
{
public:
	u_int8_t qa_type;
	
	double qa_;
};

union hdr_qrouting_all
{
	hdr_route_discovery_pkt rd;
	hdr_qrouting_pkt ph;
	hdr_qrouting_pkt_ack qa;
};
struct rtable_ent
{
public:
	nsaddr_t n;
	nsaddr_t own;
	double qenergy_;
	double qvalue_;
	double metric_;
};

class qrouting_rtable		
{
	friend class rtable_ent;
public:
	qrouting_rtable();
	void add_entry(nsaddr_t,nsaddr_t,double,double,double);
	rtable_ent* lookup(nsaddr_t);
	nsaddr_t minq(nsaddr_t);
	void update_t(nsaddr_t,double);
	void print();
	rtable_ent* rtab;
	int count;

	
};

class Qrouting;

class Qrouting_PktTimer: public TimerHandler    		
{
public: 
	Qrouting_PktTimer(Qrouting* agent): TimerHandler()
	{
		agent_ = agent;
	}
protected:
	Qrouting* agent_;
	virtual void expire(Event* e); 			
};

class Qrouting_Ack_PktTimer: public TimerHandler    		
{
public: 
	Qrouting_Ack_PktTimer(Qrouting* agent): TimerHandler()
	{
		agent_ = agent;
	}
protected:
	Qrouting* agent_;
	virtual void expire(Event* e); 			
};


class Qrouting: public Agent
{
public:
	friend class Qrouting_PktTimer;

	friend class Qrouting_Ack_PktTimer;
	
	nsaddr_t ra_addr_;		

	nsaddr_t s_dest;
	
	int accessible_var_;

	double xpos;
	
	double ypos;

	double zpos;

	MobileNode *iNode;

	double iEnergy;

	qrouting_rtable rtable_;

	PortClassifier* dmux_; 		//For passing packets to agent
	Trace* logtarget_;		//For logging routing table files. By default only packet tracing done.
	Qrouting_PktTimer pkt_timer_; 	//obj for accessing timer methods
	
	Qrouting_Ack_PktTimer pkt_ack_timer_;
	void send_route_discovery();			

	void reset_qrouting_pkt_timer();

	void reset_qrouting_pkt_ack_timer();

	void send_qrouting_pkt();
		
	void send_qrouting_ack();
	
	void recv_qrouting_pkt(Packet*);
	
	void forward_data(Packet*);

	void  recv_qrouting_ack(Packet*);			

	Qrouting(nsaddr_t);

	int command(int,const char*const*);

	void recv(Packet*,Handler*);	

	void recv_route(Packet*);

	void send_qrouting_ack1(nsaddr_t dest);

	
};

//IMPLEMENTATION

qrouting_rtable::qrouting_rtable()
{
	rtab = new rtable_ent[10];
	count = 0;
}

void qrouting_rtable::print()
{
	rtable_ent temp;
	for(unsigned int i = 0; i < count; i++)
	{
		temp = rtab[i];
	printf("S_Node = %d\tNeig_Node = %d\tEnergy=%f\tMetric=%f\tQvalue=%f\n",temp.own,temp.n,temp.qenergy_,temp.metric_,temp.qvalue_);
	}
}

void qrouting_rtable::add_entry(nsaddr_t own,nsaddr_t n,double e,double m, double q)
{
	rtab[count].own = own;
	rtab[count].n = n;
	rtab[count].qenergy_ = e;
	rtab[count].metric_ = m;
	rtab[count].qvalue_ = q;
	count++;
	
}
rtable_ent* qrouting_rtable::lookup(nsaddr_t n)
{
	rtable_ent temp;
	for(unsigned int i = 0; i < count; i++)
	{
		temp = rtab[i];
		if(temp.n == n)
		{
			return &temp;
		}
	}
}

nsaddr_t qrouting_rtable::minq(nsaddr_t source)
{
	rtable_ent temp;
	rtable_ent min;
	for(int i = 0; i < count; i++)
	{
		temp = rtab[i];
		if(temp.n != source)
		{	
			min = rtab[i];
			break;
		}
	}
	for(int i = 0; i < count; i++)
	{
		temp = rtab[i];
		if(temp.n != source && temp.qvalue_ < min.qvalue_)
		{
			min = temp;
		}
	}
	return min.n;
}
void qrouting_rtable::update_t(nsaddr_t n, double e)
{
	double alpha = 0.5;
	rtable_ent temp;
	for(int i = 0; i < count; i++)
	{
		temp = rtab[i];
		if(temp.n == n)
		{
			rtab[i].qenergy_ = e;
			double qold = rtab[i].qvalue_;
			rtab[i].qvalue_ = ((1-alpha)*qold+alpha*rtab[i].metric_)*exp(1.0/(rtab[i].qenergy_));
		}
	}
 
}

int hdr_qrouting::offset_;
class QroutingHeaderClass: public PacketHeaderClass		//tcl binding of new packet header
{
	public:
		QroutingHeaderClass(): PacketHeaderClass("PacketHeader/Qrouting",sizeof(hdr_qrouting_all))
		{
			bind_offset(&hdr_qrouting::offset_);
		}
}class_rtProtoQrouting_hdr;

static class QroutingClass: public TclClass			//tcl binding of new agent
{
public:
	QroutingClass(): TclClass("Agent/Qrouting")
	{
	
	}
	TclObject* create(int argc, const char*const* argv)
	{
		assert(argc == 5);
		return (new Qrouting((nsaddr_t)Address::instance().str2addr(argv[4])));
	}
}class_rtProtoQrouting;

void Qrouting_PktTimer::expire(Event* e)   
{
	agent_->send_qrouting_pkt();
	agent_->reset_qrouting_pkt_timer();
}


void Qrouting_Ack_PktTimer::expire(Event* e)   	
{
	//printf("Expired\n");
	agent_->send_qrouting_ack();
	//printf("Pack qrouting sent");
	agent_->reset_qrouting_pkt_ack_timer();
}

Qrouting::Qrouting(nsaddr_t id): Agent(PT_QROUTING), pkt_timer_(this), pkt_ack_timer_(this)    
{
	bind_bool("accessible_var_",&accessible_var_);
	ra_addr_ = id;
	
	xpos = 0.0;
	ypos = 0.0;
	zpos = 0.0;
	MobileNode *iNode;
	iEnergy = 0.0;
}

int Qrouting::command(int argc, const char*const* argv)		
{
	if(argc == 2)
	{
		if(strcasecmp(argv[1],"start") == 0)			
		{	
			send_route_discovery();
			pkt_timer_.resched(1.0);
			pkt_ack_timer_.resched(1.1);
			return TCL_OK;
		}		
		else if (strcasecmp(argv[1],"print_rtable") == 0)
		{
			rtable_.print();
			return TCL_OK;
		}	
	}

	else if (argc == 3)
	{
		if (strcmp(argv[1], "port-dmux") == 0) 
		{
    			dmux_ = (PortClassifier *)TclObject::lookup(argv[2]);
			if (dmux_ == 0) 
			{
				fprintf (stderr, "%s: %s lookup of %s failed\n", __FILE__,
				argv[1], argv[2]);
				return TCL_ERROR;
			}
			return TCL_OK;
    		}
		else if(strcmp(argv[1], "log-target") == 0 || strcmp(argv[1], "tracetarget") == 0) 
		{
      			logtarget_ = (Trace*) TclObject::lookup(argv[2]);
      			if(logtarget_ == 0)
				return TCL_ERROR;
      			return TCL_OK;
    		}
	}	
	return Agent::command(argc,argv);  
	
}						

void Qrouting::send_route_discovery()
{
		
	Packet* p = allocpkt();
	struct hdr_ip* ih = HDR_IP(p);
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_route_discovery_pkt* rd = HDR_ROUTE_DISCOVERY_PKT(p);	

	iNode = (MobileNode*)(Node::get_node_by_address(ra_addr_));

	rd -> qenergy_ = iNode -> energy_model() -> energy();
	rd -> rd_type = QROUTING_ROUTE_DISC;
	
	ch -> ptype() = PT_QROUTING;
	ch -> direction() = hdr_cmn::DOWN;
	ch -> size() = IP_HDR_LEN + 7;
	ch -> error() = 0;
	ch -> next_hop() = IP_BROADCAST;
	ch -> addr_type() = NS_AF_INET;

	ih -> saddr() = ra_addr_;
	ih -> daddr() = IP_BROADCAST;
	ih -> sport() = RT_PORT;
	ih -> dport() = RT_PORT;
	ih -> ttl() = IP_DEF_TTL;
	
	Scheduler::instance().schedule(target_,p,JITTER);

} 

void Qrouting::recv(Packet *p, Handler *h)	
{		
	struct hdr_qrouting* qh = HDR_QROUTING(p);

	switch(qh -> qh_type)
	{

		case QROUTING_ROUTE_DISC:
			recv_route(p);
			break;
		case QROUTING_PKT:
			recv_qrouting_pkt(p);
			break;
		case QROUTING_ACK:
			recv_qrouting_ack(p);		
			break;
		default:
			break;
		
	}				

}

void Qrouting::reset_qrouting_pkt_timer()
{
	pkt_timer_.resched((double)0.005);
}

void Qrouting::reset_qrouting_pkt_ack_timer()
{
	pkt_ack_timer_.resched((double)5.0);
}

void Qrouting::recv_route(Packet* p)
{
	struct hdr_route_discovery_pkt* rd = HDR_ROUTE_DISCOVERY_PKT(p);
	struct hdr_ip* ih = HDR_IP(p);

	iNode = (MobileNode*)(Node::get_node_by_address(ih->saddr()));

	double x1 = iNode->X();
	double y1 = iNode->Y();

	iNode = (MobileNode*)(Node::get_node_by_address(ra_addr_));

	double x2 = iNode->X();
	double y2 = iNode->Y();

	double dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
	
	double metric = dist/70.7;
	
	if(rd->qenergy_ >= 0.0001)
	{

		double qvalue = metric*exp(1.0/(rd->qenergy_));
	
		rtable_.add_entry(ra_addr_,ih->saddr(),rd->qenergy_,metric,qvalue);
	}

	Packet::free(p);

}

void Qrouting::send_qrouting_pkt()
{
	MobileNode* source =(MobileNode*) Node::get_node_by_address(0);
	nsaddr_t sou = source -> nodeid();

	if(sou == ra_addr_)
	{

		Packet* p = allocpkt();
		struct hdr_qrouting_pkt *qt = HDR_QROUTING_PKT(p);
		struct hdr_cmn *ch = HDR_CMN(p);
		struct hdr_ip* ih = HDR_IP(p);

		qt -> q_type = QROUTING_PKT;

		nsaddr_t dest = rtable_.minq(sou);
		
		ch -> ptype() = PT_QROUTING;
		ch -> direction() = hdr_cmn::DOWN;
		ch -> size() = IP_HDR_LEN + 7;
		ch -> error() = 0;
		ch -> next_hop() = dest;
		ch -> addr_type() = NS_AF_INET;

		MobileNode* sink =(MobileNode*) Node::get_node_by_address(5);
		nsaddr_t s = sink -> nodeid();
		
		ih -> saddr() = sou;
		ih -> daddr() = s;		
		ih -> sport() = RT_PORT;
		ih -> dport() = RT_PORT;
		ih -> ttl() = IP_DEF_TTL;

		Scheduler::instance().schedule(target_,p,JITTER);
	}	
}

void Qrouting::send_qrouting_ack()
{
	Packet* p = allocpkt();
	struct hdr_qrouting_pkt_ack *qa = HDR_QROUTING_PKT_ACK(p);
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	
	iNode = (MobileNode*)(Node::get_node_by_address(ra_addr_));
	qa -> qa_type = QROUTING_ACK;
	qa -> qa_ = iNode->energy_model()->energy();

	ch -> ptype() = PT_QROUTING;
	ch -> direction() = hdr_cmn::DOWN;
	ch -> size() = IP_HDR_LEN + 7;
	ch -> error() = 0;
	ch -> next_hop() = IP_BROADCAST;
	ch -> addr_type() = NS_AF_INET;

	ih -> saddr() = ra_addr_;
	ih -> daddr() = IP_BROADCAST;				
	ih -> sport() = RT_PORT;
	ih -> dport() = RT_PORT;
	ih -> ttl() = IP_DEF_TTL;

	Scheduler::instance().schedule(target_,p,JITTER);
}
void Qrouting::send_qrouting_ack1(nsaddr_t dest)
{
	Packet* p = allocpkt();
	struct hdr_qrouting_pkt_ack *qa = HDR_QROUTING_PKT_ACK(p);
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	
	iNode = (MobileNode*)(Node::get_node_by_address(ra_addr_));
	qa -> qa_type = QROUTING_ACK;
	qa -> qa_ = iNode->energy_model()->energy();

	ch -> ptype() = PT_QROUTING;
	ch -> direction() = hdr_cmn::DOWN;
	ch -> size() = IP_HDR_LEN + 7;
	ch -> error() = 0;
	ch -> next_hop() = dest;
	ch -> addr_type() = NS_AF_INET;

	ih -> saddr() = ra_addr_;
	ih -> daddr() = dest;				
	ih -> sport() = RT_PORT;
	ih -> dport() = RT_PORT;
	ih -> ttl() = IP_DEF_TTL;

	Scheduler::instance().schedule(target_,p,JITTER);
}


void  Qrouting::recv_qrouting_pkt(Packet* p)
{
	struct hdr_qrouting_pkt *qt = HDR_QROUTING_PKT(p);
	struct hdr_ip* ih = HDR_IP(p);

	MobileNode* sink =(MobileNode*) Node::get_node_by_address(5);

	nsaddr_t s = sink -> nodeid();

	if(ra_addr_ == s)
	{
		Tcl& tcl = Tcl::instance();
		Packet::free(p);
	}
	else
	{	
		send_qrouting_ack1(ih->saddr());
		forward_data(p);
	}	
	
}
void Qrouting::forward_data(Packet* p)
{
	MobileNode* sink =(MobileNode*) Node::get_node_by_address(5);
	nsaddr_t s = sink -> nodeid();

	if(ra_addr_ == s)
	{
		Packet::free(p);
	}

	else
	{
		
		struct hdr_qrouting_pkt *qt = HDR_QROUTING_PKT(p);
		struct hdr_cmn *ch = HDR_CMN(p);
		struct hdr_ip* ih = HDR_IP(p);

		qt -> q_type = QROUTING_PKT;
	
		nsaddr_t dest = rtable_.minq(ih->saddr());

		ch -> ptype() = PT_QROUTING;
		ch -> direction() = hdr_cmn::DOWN;
		ch -> size() = IP_HDR_LEN + 7;
		ch -> error() = 0;
		ch -> next_hop() = dest;
		ch -> addr_type() = NS_AF_INET;

		ih -> saddr() = ra_addr_;
		ih -> daddr() = s;		
		ih -> sport() = RT_PORT;
		ih -> dport() = RT_PORT;
		ih -> ttl() = IP_DEF_TTL;

		Scheduler::instance().schedule(target_,p,JITTER);
	}
}
void Qrouting::recv_qrouting_ack(Packet* p)
{
	struct hdr_qrouting_pkt_ack *qa = HDR_QROUTING_PKT_ACK(p);
	struct hdr_ip* ih = HDR_IP(p);

	rtable_ent* rt = rtable_.lookup(ih->saddr());
	
	double en = qa -> qa_;	

	if(en >= 0.0001)
		rtable_.update_t(rt->n,en);

}
