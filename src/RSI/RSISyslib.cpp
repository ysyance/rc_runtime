
#include "RSISyslib.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

bool RSIStopFlag = true; /* THIS IS VERY IMPORTANT, WHICH CONTROL THE WHOLE LIFECYCLE OF RSI */

#define RSI_DEBUG

#ifdef RSI_DEBUG_PRINT
std::unordered_map<int, std::string> rdataIndexMap;   // index --> var
#endif

inline int rsi_pid(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	EntityPID *entity = dynamic_cast<EntityPID*>(config);
	if(entity != NULL) {
#ifdef RSI_DEBUG
		entity->printInfo();
#endif
	} else {
		std::cout << "this fb does not have config entity" << std::endl;
	}
	return 0;
}

inline int rsi_comm_interface( 	std::vector<int>& params, 
								EntityBase* config, 
								std::vector<IValue>& addrspace) 
{
	EntityComm *entity = dynamic_cast<EntityComm*>(config);
	if(entity != NULL) {
#ifdef RSI_DEBUG
		entity->printInfo();
#endif
	} else {
		std::cout << "this fb does not have config entity" << std::endl;
		return -1;
	}

    // struct sockaddr_in addr;
    int &sockfd = entity->sockfd;

    if(entity->initflag != true) {
    	sockfd = socket(AF_INET, SOCK_DGRAM, 0); 

    	struct timeval timeout = {0, 12};
    	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));

	    bzero(&entity->addr, sizeof(entity->addr));
	    entity->addr.sin_family = AF_INET;
	    entity->addr.sin_port = htons(atoi(entity->port.c_str()));
	    entity->addr.sin_addr.s_addr = inet_addr(entity->ip.c_str());

	    entity->initflag = true;
    }

    entity->xmlGenerate(addrspace);
    std::cout << entity->sendBuffer << std::endl;
    int sn = sendto(sockfd, entity->sendBuffer, strlen(entity->sendBuffer), 0, (struct sockaddr *)&entity->addr, sizeof(entity->addr));
    std::cout << "send ==> " << sn << " bytes" << std::endl;

/*    std::string str = "<?xml version=\"1.0\" encoding=\"GBK\"?>\
    <Root>\
          <RKorr X=\"1.2\" Y=\"2.3\" Z=\"3.4\" A=\"4.5\" B=\"5.6\" C=\"6.7\"/>\
          <FREE>100</FREE>\
          <STOP stopFlag=\"0\" />\
	<Root>";  
	sprintf(entity->recvBuffer, "%s", str.c_str());
	std::cout << entity->recvBuffer << std::endl;*/

    std::cout << "waiting for data ..." << std::endl;
    int rn = recvfrom(sockfd, entity->recvBuffer, 4096, 0, NULL, NULL);
    if(rn == -1 && errno == EAGAIN) {
    	std::cout << "receive timeout " << std::endl;
    	throw rc_rsicomm_outoftime_exception("rsi_comm_interface");
    } else {
    	std::cout << "recv <== " << rn << " bytes" << std::endl;
    	entity->xmlParse(addrspace);
    }
    

    // close(sockfd);
    
	return 0;
}



inline int rsi_poscorr(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	EntityPOSCORR *entity = dynamic_cast<EntityPOSCORR*>(config);
	if(entity != NULL) {
#ifdef RSI_DEBUG
		entity->printInfo();
#endif
	} else {
		std::cout << "this fb does not have config entity" << std::endl;
	}
	return 0;
}


inline int rsi_axiscorr(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	EntityAXISCORR *entity = dynamic_cast<EntityAXISCORR*>(config);
	if(entity != NULL) {
#ifdef RSI_DEBUG
		entity->printInfo();
#endif
	} else {
		std::cout << "this fb does not have config entity" << std::endl;
	}	
	return 0;
}
