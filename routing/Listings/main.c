//-----------------------
// Wireless Communication Design Challenge 2
// Author List: 
// joseph(D05921016@ntu.edu.tw), 
// Wei-Che Chen (r05942110@ntu.edu.tw)
// Chin yen su (james821007@gmail.com) 
// Version: 1.2
// Date: January 14,2017
//---------------------
// Note: 
// LED Pin on PA5(J3-16) 
//------------------------------

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "autonet.h"

//--------------------
#define MY_DEVICE_ADDR  0x0007//!!!!! SHOULD set different mac addr  !!!!!!!!
#define SINK_ADDR 0x0012
#define EMITTER_ADDR 0x0001   // periodically sent  (200ms), set by TA
#define DEBUG 0
//------------------------------


#define PAN_ID  0x00AA   //same with TA code
#define RADIO_CHANNEL  18 //same with TA code
#define TYPE 0x1

#define RCV_BUFSIZE 128
#define MAX 30  //routing table size
//define Packet type
#define RREQ 				0x80
#define RREP 			  0x40
#define FLAG   			0x20
#define DATA   		0x10
#define DATA_ACK  0x11

#define BLINK_PERIOD 300  //unit: ms


#define T 200 // unit : ms

#define EN_ROUTE_LIFE 1
#define RENEW_INTERVAL (5000/(T))  //5 seconds
#define LIFE_TIME (RENEW_INTERVAL*2)
uint16_t renew_interval;  // unit : ms

int debug=DEBUG;

typedef struct _host
{
  uint16_t my_ID;  
	uint16_t my_addr;
		
}Host;

typedef struct _packet
{
  uint8_t type;
  uint32_t id;
  uint16_t to; //to 
  uint16_t from; //from
 	uint16_t hop_count;
  uint8_t length; // data length
  uint8_t data[5]; // [0]0x5 [1]0x30 [2]sequence number [3]isACK [4]payload
}Packet;


	  

typedef struct _route {
  
  uint16_t dest_mac;
  uint16_t next_mac;
	uint16_t hop_count;
	uint16_t life_time;
	uint32_t src_seq;
	
}Route;

typedef struct _rtable {
        
  Route table[MAX];
  uint8_t index; 

}Route_Table;


Host host;
uint8_t finish=0;
#define PRINT_BUFSIZE 128
char output_array[PRINT_BUFSIZE]={0}; 	
Route_Table rtable;
uint16_t Dest_Addr;
uint16_t Src_Addr;
uint32_t unique_bid;
uint16_t timer_count=0;
uint16_t renew_routing=0;
//Function list
//-------------------
void debug_print(char *s);
void show_myinfo(void);
void dump_packet(Packet *p);
void broadcast_RREQ(uint16_t addr);
void Re_broadcast_RREQ(Packet *p);
void send_message( uint8_t type,uint16_t to,uint16_t from, uint8_t *data, uint8_t size);
void send_RREP(Packet *p);
void send_DATA(uint16_t id, uint8_t *data, uint8_t size,Route_Table *tbl);
void send_DATA_ACK(uint16_t id, uint8_t *data, uint8_t size,Route_Table *tbl);
void init(void);

//---Routing Table Processing-----------

void init_table(Route_Table *tbl);
uint8_t add_route(Route *route,Route_Table *tbl);
void update_life_time(uint16_t dest, Route_Table *tbl);
void dump_table(Route_Table *tbl);
Route * find_next_hop(uint16_t addr, Route_Table *tbl);
Route * find_duplicate(uint16_t addr1,uint16_t addr2,Route_Table *tbl);
Route * find_vacancy(Route_Table *tbl);
void renew_route(Route_Table *tbl);
//----Utility function---------------------
void blink_led(int gpio_index,int count,uint32_t delay);

int main(void)
{
	
	uint8_t rcvd_msg[RCV_BUFSIZE];
	uint8_t rcvd_payload[RCV_BUFSIZE];
	uint8_t rcvd_length;
 

	uint8_t rcvd_payloadLength;
	uint8_t rcvd_rssi;
	uint8_t data_out[2];
	Packet * pkt;
	Route r_entry={0};	
	

  Route *r;

	init();  //init all
  init_table(&rtable);
	
   debug_print("Starting Design Challenge 2.....\r\n");
	 show_myinfo();
   setTimer(1, T, UNIT_MS);  // T ms

	timer_count=0;
	
	while(1){
		
		 if(checkTimer(1)) { //timer
        	 

			     timer_count++;
			     renew_interval--;
			 
			 r=find_next_hop(SINK_ADDR,&rtable);	
			 //if you know the SINK addr , raise your hand
			 if (r!=NULL) setGPIO(5,1);
			 else setGPIO(5,0);
			
		 
		 
			#if EN_ROUTE_LIFE==1

			  //dump_table(&rtable);
			 if (renew_interval==0 ) {			
		 			renew_interval=RENEW_INTERVAL;
				  renew_route(&rtable);
				}
			#endif
				
      } 
	
		if(RF_Rx(rcvd_msg, &rcvd_length, &rcvd_rssi)){
			
			getDestAddr(data_out, rcvd_msg); 
			Dest_Addr=*(uint16_t *)data_out; //get the dest mac address
			getSrcAddr(data_out, rcvd_msg); 
			Src_Addr=*(uint16_t *)data_out;   //get the src mac address
			

			getPayloadLength(&rcvd_payloadLength, rcvd_msg);
			getPayload(rcvd_payload, rcvd_msg, rcvd_payloadLength);
			
			//get packet from emitter. we should sent to Sink device, but we should know the route in advance
			if (Src_Addr==EMITTER_ADDR) {		
				 #if DEBUG==1 
          uint8_t i;				
	        for (i=0;i<rcvd_payloadLength;i++) {
	       	sprintf((char *)output_array,"%#x ",rcvd_payload[i]);
				  debug_print(output_array);					
	 
			   }					
				  debug_print("\r\n");			
				 #endif
				 
				 /*
           if (host.my_addr==SINK_ADDR) {
					       //for direct-radio-link case,just send back diectly,no need to ask others 
						     rcvd_payload[3]=1; //set isACK=1
						     RF_Tx(EMITTER_ADDR,rcvd_payload,rcvd_payloadLength);
				         blink_led(1,200);
							 
						 

           } else {		
           */				 
								//i'm not the sink device. do i know the dest. addr ? 
				       if (debug) dump_table(&rtable);		
								r=find_next_hop(SINK_ADDR,&rtable);	
								if (r==NULL && SINK_ADDR!=host.my_addr) { //if not found and not myself, broadcast RREQ 
					
										  sprintf((char *)output_array,"Broadcast_RREQ to find %#x \r\n",SINK_ADDR);
							 	      debug_print(output_array);
 										 	broadcast_RREQ(SINK_ADDR);											  
								}
								
								if (r!=NULL) { 
								
											debug_print("I know the route to Sink device, so just send DATA \r\n");
									    send_message(DATA,SINK_ADDR,host.my_addr,(uint8_t *)rcvd_payload,rcvd_payloadLength);
								}		
				// }
          continue; 							
			 
			 } else { 
					
		      pkt=(Packet *)rcvd_payload;
			    if (debug) {
							debug_print("Rx ");
							dump_packet(pkt);
					}
		  }
			 
		  update_life_time(Src_Addr,&rtable);

			//packet handling
			switch (pkt->type) {
 
	      	case RREQ:
						
		
							
							if (pkt->to==host.my_addr) {  //that's me! reply RREP to the original of RREP invoker 
							   sprintf((char *)output_array,"Yes, I'm %#x, so replay RREP back to %#x\r\n",pkt->to,pkt->from);
							   debug_print(output_array);
							   send_RREP(pkt);
						 	} else {		
								  	
					     //do i know the dest. addr ? 
								r=find_next_hop(pkt->to,&rtable);	
								if (r==NULL) { 
									
									if ((pkt->id>>16)==host.my_addr) {  //prevent loop (broadcast storm)
										   debug_print("skip it\r\n");																						
								  } else {
										
									//   Re_broadcast_RREQ(pkt);				
                      								
                      r=find_next_hop(pkt->from,&rtable);	
                     	if (r==NULL || (r!=NULL && r->src_seq!=pkt->id)) {
												    sprintf((char *)output_array,"%#x not found, Re_broadcast_RREQ\r\n",pkt->to);
							 	             debug_print(output_array);						
													 	 Re_broadcast_RREQ(pkt);		//if not found, broadcast RREQ 		
 										 	   
										  }
										
									}									
			 
								} else {  //yes, I know the route
									 // comment it , if you want to always say "I don't know "
								      sprintf((char *)output_array,"I know the route to %#x \r\n",pkt->to);
							 	      debug_print(output_array);
						          send_RREP(pkt);
									
								   
								}
						  }
      
							 //-----joseph move to here, Jan 16, 2017----------------------------
							//create reverse path
					   if (pkt->from!=host.my_addr) { 
									r_entry.dest_mac=pkt->from;
									r_entry.next_mac=Src_Addr;
									r_entry.hop_count=++pkt->hop_count;
							    r_entry.src_seq=pkt->id; //prevent loop
					        add_route(&r_entry,&rtable);			
                  dump_table(&rtable);
								 }
				  //-----------------------------------------
					
		
					break;
				
				case RREP:
				 //update routing table 	
            debug_print("Get RREP...\r\n");
						r_entry.dest_mac=pkt->from;
						r_entry.next_mac=Src_Addr;
						r_entry.hop_count=++pkt->hop_count;
				    
						add_route(&r_entry,&rtable);			
            dump_table(&rtable);			
						
				    if (pkt->to != host.my_addr) {
							debug_print("send back RREP...\r\n");
			         send_RREP(pkt);
						} else {
						   debug_print("I received RREP, so we have known the route\r\n");
	
						}
				
			
				
	        break;			
  
				case DATA:			
		 					 
						if (pkt->to==host.my_addr) { 
				        sprintf((char *)output_array,"Get DATA data with seq=%#x\r\n",pkt->data[2]);
						    debug_print(output_array);
							  debug_print("Send ACK back\r\n");
							  pkt->data[3]=1; //isACK=1 
							  send_message(DATA_ACK,pkt->from,host.my_addr,pkt->data,pkt->length);
						  	blink_led(4,1,200); 						
							
							 if (pkt->data[2]==0x13) {
							    //finish
								     dump_table(&rtable);	
							    	blink_led(5,30,50); 
								    setGPIO(4,1);
                 } 						 
						} else { //not owner , so just forward
									debug_print("Forward  DATA packet\r\n");
						       //change 'from addr' along the path
							   	send_message(DATA,pkt->to,host.my_addr,pkt->data,pkt->length);
		
				    }
						
						 blink_led(5,2,50); 
				  break;

				case DATA_ACK:			
					 					 
						if (pkt->to==host.my_addr) { 
							  debug_print("I received ACK, then send data to the emitter dietcly\r\n");
								RF_Tx(EMITTER_ADDR,pkt->data,5);
					
						} else {
								  debug_print("Forward ACK packet\r\n");
						     	send_message(DATA_ACK,pkt->to,host.my_addr,pkt->data,pkt->length);
								
				    }
						
						  blink_led(5,2,50); 
				  break;

						
					
					default:
						//unknown type
					  break;
						
			 } //end switch
	   }  //if rx available
				
	}  //edn while(1)

} //end main




//---------------------------------------------
//Packet Handling
//---------------------------------------------
void init()
{
	Initial(MY_DEVICE_ADDR,TYPE, RADIO_CHANNEL, PAN_ID);
	
  host.my_addr=MY_DEVICE_ADDR;
	host.my_ID='@'+MY_DEVICE_ADDR; //init to '@':0x40  
  unique_bid=(host.my_addr&0xff)<<16; //unique_bid= host.my_ID (16bit) + seq no. (16bit)
  renew_interval=RENEW_INTERVAL;

	
	
}

void debug_print(char *s)
{
	  int len=strlen(s);
	  COM2_Tx((uint8_t *)s,len);
	
}

void broadcast_RREQ(uint16_t addr)
{  	
  	
	 Packet packet={0};
	 if (addr==host.my_addr) return;
	 unique_bid++;
	 packet.id=unique_bid; //unique broadcast ID
	 packet.type=RREQ; //route request
	 packet.to=addr; // you want to find the "addr"
	 packet.from=host.my_addr;	
   packet.hop_count=0;
	//broadcasting
   RF_Tx(0xFFFF,(uint8_t *)&packet,sizeof(Packet));

}


void Re_broadcast_RREQ(Packet *p)
{  	

  	//broadcasting
	p->hop_count++;
  RF_Tx(0xFFFF,(uint8_t *)p,sizeof(Packet));

}

void send_message( uint8_t type,uint16_t to,uint16_t from, uint8_t *data, uint8_t size)
{
 
	 Route *r;
	 Packet packet={0};
	 packet.type=type; 

	 packet.to=to; 
	 packet.from=from;	
	 memcpy(&packet.data,data,size);
	 packet.length=size;
	 	
  r=find_next_hop(to,&rtable);   
	//if (debug) dump_packet(&packet);
 	if (r)    //unicast
     RF_Tx(r->next_mac,(uint8_t *)&packet,sizeof(Packet));
 
}


void send_DATA(uint16_t id, uint8_t *data, uint8_t size,Route_Table *tbl)
{

	 Route *r;
	
  r=find_next_hop(id,tbl);   
 	if (r)
	   send_message(DATA,id,r->next_mac ,data,size);

}

void send_DATA_ACK(uint16_t id, uint8_t *data, uint8_t size,Route_Table *tbl)
{

	 Route *r;
	
  r=find_next_hop(id,tbl);   
 	if (r)
	   send_message(DATA_ACK,id,r->next_mac ,data,size);

}


void send_RREP(Packet *p)
{
  Route *r;
	Packet packet={0};

 
  
	packet.type=RREP;
	packet.to=p->from;
	packet.from=p->to;
	packet.hop_count=0;
	
	 r=find_next_hop(packet.to,&rtable);
	 sprintf((char *)output_array,"send RREP to %#x through %#x\r\n",p->from,r->next_mac);
	 debug_print(output_array);
 	if (r) //unicast
	  RF_Tx(r->next_mac,(uint8_t *)&packet,sizeof(Packet));
	
}

void dump_packet(Packet *p)
{
	
	 uint8_t i;
  
	if (debug) {
		sprintf((char *)output_array,"DUMP:  %#x  %#x  %#x  %#x  %#x  %#x %d\r\n==>",p->id,Dest_Addr,Src_Addr,p->type,p->to,p->from,p->hop_count);
	   	debug_print(output_array);
	     for (i=0;i<p->length;i++) {
	       	sprintf((char *)output_array,"%#x ",p->data[i]);
				  debug_print(output_array);
	 
			 }
    debug_print("\r\n");
		 }
}


void show_myinfo()
{

  sprintf((char *)output_array,"My Info: ID=%c,MAC Addr=%#x\r\n",host.my_ID,host.my_addr);
	debug_print(output_array);
	
}

//------------------------------
//Utility function
//--------------------------------

//blinking LED
void blink_led(int gpio_index,int count,uint32_t delay)
{
	uint8_t i;
	for (i=0;i<count;i++) {		
	    setGPIO(gpio_index,1);
		  Delay(delay);
	  	setGPIO(gpio_index,0);
		  Delay(delay);
		
  }		
}

//---------------------------------------------
//Table Handling
//---------------------------------------------
void init_table(Route_Table *tbl)
{
 
   memset(tbl,0,sizeof(Route_Table));
   tbl->index=0;

}

uint8_t add_route(Route *route,Route_Table *tbl)
{

	 Route *r;
	 r=find_duplicate(route->dest_mac,route->next_mac,tbl);
	 route->life_time=timer_count;	 
	 if (r!=NULL) { //already exist , just update route info
		  if (route->hop_count< r->hop_count) { 
				 r->hop_count=route->hop_count;
				 r->life_time=route->life_time;
        
      }				
	 } else if((r=find_vacancy(tbl))!=NULL){ //vacancy entry
	
     //insert new entry at vacancy 
		  memcpy(r,route,sizeof(Route));
		 
	 }else { //new entry
		 memcpy(&tbl->table[tbl->index],route,sizeof(Route));
     tbl->index++;
	 }
	 
   return tbl->index;
 
}

Route * find_next_hop(uint16_t addr, Route_Table *tbl)
{
   uint8_t i;

   for (i=0;i<tbl->index;i++)
   {
       
       if (addr==tbl->table[i].dest_mac) 
				   return &tbl->table[i];    
           
   }

   return NULL;

}

void update_life_time(uint16_t dest, Route_Table *tbl)
{

  uint8_t i;

   for (i=0;i<tbl->index;i++)
   {
       
       if (dest==tbl->table[i].dest_mac) 
				  tbl->table[i].life_time=timer_count;
           
   }


}



void renew_route(Route_Table *tbl)
{
   uint8_t i;


   for (i=0;i<tbl->index;i++)
   {
          
		 if (timer_count > (tbl->table[i].life_time+LIFE_TIME)) { //llfe time : 10 seconds
        
		     sprintf((char *)output_array,"timer_count=%d,life_time=%d,LIFE_TIME=%d\r\n",timer_count,tbl->table[i].life_time,LIFE_TIME);
		     debug_print(output_array);
			   memset(&tbl->table[i],0,sizeof(Route));    
		 }			 
   }
     if (debug) dump_table(&rtable);		
}


Route * find_duplicate(uint16_t addr1,uint16_t addr2,Route_Table *tbl)
{
   uint8_t i;

   for (i=0;i<tbl->index;i++)
   {
       
       if (addr1==tbl->table[i].dest_mac && addr2==tbl->table[i].next_mac) 
            return &tbl->table[i];    
   }

   return NULL;

}

Route * find_vacancy(Route_Table *tbl)
{
   uint8_t i;

   for (i=0;i<tbl->index;i++)
   {
       
       if (tbl->table[i].dest_mac==0) 
            return &tbl->table[i];    
   }

   return NULL;

}


void dump_table(Route_Table *tbl)
{
   uint8_t i;
   debug_print("dest_mac  next_mac hop_count life_time Src_seq\r\n");    
	
   for (i=0;i<tbl->index;i++)
   {
		 	sprintf((char *)output_array,"0x%x\t0x%x %d %d %#x\r\n",tbl->table[i].dest_mac,tbl->table[i].next_mac,tbl->table[i].hop_count,tbl->table[i].life_time,tbl->table[i].src_seq);
	   	debug_print(output_array);
      
   }

}



