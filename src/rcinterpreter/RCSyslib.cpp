#include "RCSyslib.h"


inline int rc_communication(std::vector<int>& params, RCEntityBase* config, RC_SymbolTable& symTable){
	if(params.size() != 1 || symTable.addrspace[params[0]].type != TINT) {
        throw rc_wrongfuncparams_exception("RC_COMMUNICATION");
    }
    RCCommEntity *entity = dynamic_cast<RCCommEntity*>(config);
	if(entity != NULL) {
		entity->printInfo();
	} else {
		std::cout << "this fb does not have config entity" << std::endl;
		throw rc_fb_lackofconfig_exception("RC_COMMUNICATION");
	}

    // struct sockaddr_in addr;
    int &sockfd = entity->sockfd;

    if(entity->initflag != true) {
    	sockfd = socket(AF_INET, SOCK_DGRAM, 0); 

        int32_t t = symTable.addrspace[params[0]].v.value_i;
        if(t <= 0 || t > 1000000) {
            t = 12;
        }
    	struct timeval timeout = {0, 12};
        timeout.tv_usec = t;

    	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));

	    bzero(&entity->addr, sizeof(entity->addr));
	    entity->addr.sin_family = AF_INET;
	    entity->addr.sin_port = htons(atoi(entity->port.c_str()));
	    entity->addr.sin_addr.s_addr = inet_addr(entity->ip.c_str());

	    entity->initflag = true;
    }

    entity->xmlGenerate(symTable);
    std::cout << entity->sendBuffer << std::endl;
    int sn = sendto(sockfd, entity->sendBuffer, strlen(entity->sendBuffer), 0, (struct sockaddr *)&entity->addr, sizeof(entity->addr));
    std::cout << "RC send ==> " << sn << " bytes" << std::endl;

    std::cout << "RC waiting for data ..." << std::endl;
    int rn = recvfrom(sockfd, entity->recvBuffer, 4096, 0, NULL, NULL);

    if(rn == -1 && errno == EAGAIN) {
    	std::cout << "RC receive timeout " << std::endl;
    	throw rc_rsicomm_outoftime_exception("rsi_comm_interface");
    } else {
    	std::cout << "RC recv <== " << rn << " bytes" << std::endl;
    	entity->xmlParse(symTable);
    }
    

    // close(sockfd);
    
	return 0;
}

