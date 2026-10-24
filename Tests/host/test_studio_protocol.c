#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "studio_protocol.h"
static void make_line(char *line, const char *body)
{
    sprintf(line,"%s*%04X",body,Studio_Crc16((const uint8_t *)body,strlen(body)));
}
int main(void)
{
    studio_target_t target;
    studio_lease_t lease={0};
    char line[192];
    uint8_t packet[96], payload[4]={0,0,128,63};
    assert(Studio_Crc16((const uint8_t *)"123456789",9)==0x29b1);
    make_line(line,"@C1,77,1,-100,0.3,0");
    assert(Studio_ParseTarget(line,&target));
    assert(target.token==77 && target.sequence==1 && target.rpm==-100);
    line[10]^=1; assert(!Studio_ParseTarget(line,&target));
    make_line(line,"@C1,77,1,nan,0,0");assert(!Studio_ParseTarget(line,&target));
    make_line(line,"@C1,4294967296,1,0,0,0");assert(!Studio_ParseTarget(line,&target));
    make_line(line,"@C1,77,1,0,0,0,1");assert(!Studio_ParseTarget(line,&target));
    make_line(line,"@C1,77,1,inf,0,0");assert(!Studio_ParseTarget(line,&target));
    lease.token=76;Studio_LeaseArm(&lease,0xffff0000U);
    target.token=77;target.sequence=0xffffffffU;
    assert(Studio_LeaseAccept(&lease,&target,0xffff0010U));
    assert(!Studio_LeaseAccept(&lease,&target,0xffff0011U));
    target.sequence=0;assert(Studio_LeaseAccept(&lease,&target,0x100U));
    target.sequence=0xfffffffeU;assert(!Studio_LeaseAccept(&lease,&target,0x101U));
    assert(!Studio_LeaseExpire(&lease,0x100U+299999U));
    assert(Studio_LeaseExpire(&lease,0x100U+300000U));
    target.sequence=1;assert(!Studio_LeaseAccept(&lease,&target,0x100U+300001U));
    Studio_LeaseArm(&lease,500000);assert(!Studio_LeaseAccept(&lease,&target,500010));
    assert(Studio_Encode(3,7,payload,4,packet)==14);
    assert(packet[0]=='F'&&packet[1]=='S'&&packet[6]==7&&packet[4]==4);
    assert(Studio_Crc16(packet,12)==((unsigned)packet[12]|(unsigned)packet[13]<<8));
    assert(Studio_Encode(1,0,payload,87,packet)==0);
    puts("studio protocol: CRC, strict parsing, replay, wraparound and timeout PASS");
    return 0;
}
