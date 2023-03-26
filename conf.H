/*******************************PLC_CONF.H****************************************
*传人记
*2014-05-08
*V1.0
*
******************************************************************************/
#ifndef  PLC_CONF_H
#define  PLC_CONF_H

#include "PLC_Dialogue.h"

///////////////////////////////////////////////
// STM32F103RC芯片容量限制，条件编译增减功能，如果需要测试全功能建议使用RD或以上芯片
/* 功能开启关闭 */
#define CAN_FUNC         0  
#define ADC_FUNC         1
#define DAC_FUNC         1
#define PWM_OUTPUT_FUNC  0
#define ENCODE_FUNC      1  //编码器功能，0关闭，1开启

#define RTC_FUNC         0
#define FR_FUNC          1

/* 传人记，20170601新增 */
#define POWER_DOWN_WAY   1  // 定义为1时使用线中断检测；为0时主循环检测
#define TEST_FUNC        0
#define W5500_FUNC       1
#define RS_FUNC          1
//////////////////////////////////////////////////////////////////////////////////

/* 传人记20170423新增，编码器相关 */
#define C236              PLC_RAM32(RAM_C200_ADDR + 36*4)  // X1(U/D)
#define C239              PLC_RAM32(RAM_C200_ADDR + 39*4)  // X4(U/D)
#define C251              PLC_RAM32(RAM_C200_ADDR + 51*4)  // X0(A) X1(B)
#define C253              PLC_RAM32(RAM_C200_ADDR + 53*4)  // X3(A) X4(B)

//////////////////////////////////////////////////////////////////////////////////
/*************D8000-D8255****************/     
#define D8000   plc_16BitBuf[0x0C00] 
#define D8001   plc_16BitBuf[0x0C01]
#define D8002   plc_16BitBuf[0x0C02]
#define D8003   plc_16BitBuf[0x0C03]
#define D8004   plc_16BitBuf[0x0C04]
#define D8005   plc_16BitBuf[0x0C05]
#define D8006   plc_16BitBuf[0x0C06]
#define D8007   plc_16BitBuf[0x0C07]
#define D8008   plc_16BitBuf[0x0C08]
#define D8009   plc_16BitBuf[0x0C09]
#define D8010   plc_16BitBuf[0x0C0A]
#define D8011   plc_16BitBuf[0x0C0B]
#define D8012   plc_16BitBuf[0x0C0C]
#define D8013   plc_16BitBuf[0x0C0D]
#define D8014   plc_16BitBuf[0x0C0E]
#define D8015   plc_16BitBuf[0x0C0F]
#define D8016   plc_16BitBuf[0x0C10]
#define D8017   plc_16BitBuf[0x0C11]
#define D8018   plc_16BitBuf[0x0C12]
#define D8019   plc_16BitBuf[0x0C13]
#define D8020   plc_16BitBuf[0x0C14]
#define D8021   plc_16BitBuf[0x0C15]
#define D8022   plc_16BitBuf[0x0C16]
#define D8023   plc_16BitBuf[0x0C17]
#define D8024   plc_16BitBuf[0x0C18]
#define D8025   plc_16BitBuf[0x0C19]
#define D8026   plc_16BitBuf[0x0C1A]
#define D8027   plc_16BitBuf[0x0C1B]
#define D8028   plc_16BitBuf[0x0C1C]
#define D8029   plc_16BitBuf[0x0C1D]
#define D8030   plc_16BitBuf[0x0C1E]
#define D8031   plc_16BitBuf[0x0C1F]
#define D8032   plc_16BitBuf[0x0C20]
#define D8033   plc_16BitBuf[0x0C21]
#define D8034   plc_16BitBuf[0x0C22]
#define D8035   plc_16BitBuf[0x0C23]
#define D8036   plc_16BitBuf[0x0C24]
#define D8037   plc_16BitBuf[0x0C25]
#define D8038   plc_16BitBuf[0x0C26]
#define D8039   plc_16BitBuf[0x0C27]
#define D8040   plc_16BitBuf[0x0C28]
#define D8041   plc_16BitBuf[0x0C29]
#define D8042   plc_16BitBuf[0x0C2A]
#define D8043   plc_16BitBuf[0x0C2B]
#define D8044   plc_16BitBuf[0x0C2C]
#define D8045   plc_16BitBuf[0x0C2D]
#define D8046   plc_16BitBuf[0x0C2E]
#define D8047   plc_16BitBuf[0x0C2F]
#define D8048   plc_16BitBuf[0x0C30]
#define D8049   plc_16BitBuf[0x0C31]
#define D8050   plc_16BitBuf[0x0C32]
#define D8051   plc_16BitBuf[0x0C33]
#define D8052   plc_16BitBuf[0x0C34]
#define D8053   plc_16BitBuf[0x0C35]
#define D8054   plc_16BitBuf[0x0C36]
#define D8055   plc_16BitBuf[0x0C37]
#define D8056   plc_16BitBuf[0x0C38]
#define D8057   plc_16BitBuf[0x0C39]
#define D8058   plc_16BitBuf[0x0C3A]
#define D8059   plc_16BitBuf[0x0C3B]
#define D8060   plc_16BitBuf[0x0C3C]
#define D8061   plc_16BitBuf[0x0C3D]
#define D8062   plc_16BitBuf[0x0C3E]
#define D8063   plc_16BitBuf[0x0C3F]
#define D8064   plc_16BitBuf[0x0C40]
#define D8065   plc_16BitBuf[0x0C41]
#define D8066   plc_16BitBuf[0x0C42]
#define D8067   plc_16BitBuf[0x0C43]
#define D8068   plc_16BitBuf[0x0C44]
#define D8069   plc_16BitBuf[0x0C45]
#define D8070   plc_16BitBuf[0x0C46]
#define D8071   plc_16BitBuf[0x0C47]
#define D8072   plc_16BitBuf[0x0C48]
#define D8073   plc_16BitBuf[0x0C49]
#define D8074   plc_16BitBuf[0x0C4A]
#define D8075   plc_16BitBuf[0x0C4B]
#define D8076   plc_16BitBuf[0x0C4C]
#define D8077   plc_16BitBuf[0x0C4D]
#define D8078   plc_16BitBuf[0x0C4E]
#define D8079   plc_16BitBuf[0x0C4F]
#define D8080   plc_16BitBuf[0x0C50]
#define D8081   plc_16BitBuf[0x0C51]
#define D8082   plc_16BitBuf[0x0C52]
#define D8083   plc_16BitBuf[0x0C53]
#define D8084   plc_16BitBuf[0x0C54]
#define D8085   plc_16BitBuf[0x0C55]
#define D8086   plc_16BitBuf[0x0C56]
#define D8087   plc_16BitBuf[0x0C57]
#define D8088   plc_16BitBuf[0x0C58]
#define D8089   plc_16BitBuf[0x0C59]
#define D8090   plc_16BitBuf[0x0C5A]
#define D8091   plc_16BitBuf[0x0C5B]
#define D8092   plc_16BitBuf[0x0C5C]
#define D8093   plc_16BitBuf[0x0C5D]
#define D8094   plc_16BitBuf[0x0C5E]
#define D8095   plc_16BitBuf[0x0C5F]
#define D8096   plc_16BitBuf[0x0C60]
#define D8097   plc_16BitBuf[0x0C61]
#define D8098   plc_16BitBuf[0x0C62]
#define D8099   plc_16BitBuf[0x0C63]
#define D8100   plc_16BitBuf[0x0C64]
#define D8101   plc_16BitBuf[0x0C65]
#define D8102   plc_16BitBuf[0x0C66]
#define D8103   plc_16BitBuf[0x0C67]
#define D8104   plc_16BitBuf[0x0C68]
#define D8105   plc_16BitBuf[0x0C69]
#define D8106   plc_16BitBuf[0x0C6A]
#define D8107   plc_16BitBuf[0x0C6B]
#define D8108   plc_16BitBuf[0x0C6C]
#define D8109   plc_16BitBuf[0x0C6D]
#define D8110   plc_16BitBuf[0x0C6E]
#define D8111   plc_16BitBuf[0x0C6F]
#define D8112   plc_16BitBuf[0x0C70]
#define D8113   plc_16BitBuf[0x0C71]
#define D8114   plc_16BitBuf[0x0C72]
#define D8115   plc_16BitBuf[0x0C73]
#define D8116   plc_16BitBuf[0x0C74]
#define D8117   plc_16BitBuf[0x0C75]
#define D8118   plc_16BitBuf[0x0C76]
#define D8119   plc_16BitBuf[0x0C77]
#define D8120   plc_16BitBuf[0x0C78]
#define D8121   plc_16BitBuf[0x0C79]
#define D8122   plc_16BitBuf[0x0C7A]
#define D8123   plc_16BitBuf[0x0C7B]
#define D8124   plc_16BitBuf[0x0C7C]
#define D8125   plc_16BitBuf[0x0C7D]
#define D8126   plc_16BitBuf[0x0C7E]
#define D8127   plc_16BitBuf[0x0C7F]
#define D8128   plc_16BitBuf[0x0C80]
#define D8129   plc_16BitBuf[0x0C81]
#define D8130   plc_16BitBuf[0x0C82]
#define D8131   plc_16BitBuf[0x0C83]
#define D8132   plc_16BitBuf[0x0C84]
#define D8133   plc_16BitBuf[0x0C85]
#define D8134   plc_16BitBuf[0x0C86]
#define D8135   plc_16BitBuf[0x0C87]
#define D8136   plc_16BitBuf[0x0C88]
#define D8137   plc_16BitBuf[0x0C89]
#define D8138   plc_16BitBuf[0x0C8A]
#define D8139   plc_16BitBuf[0x0C8B]
#define D8140   plc_16BitBuf[0x0C8C]
#define D8141   plc_16BitBuf[0x0C8D]
#define D8142   plc_16BitBuf[0x0C8E]
#define D8143   plc_16BitBuf[0x0C8F]
#define D8144   plc_16BitBuf[0x0C90]
#define D8145   plc_16BitBuf[0x0C91]
#define D8146   plc_16BitBuf[0x0C92]
#define D8147   plc_16BitBuf[0x0C93]
#define D8148   plc_16BitBuf[0x0C94]
#define D8149   plc_16BitBuf[0x0C95]
#define D8150   plc_16BitBuf[0x0C96]
#define D8151   plc_16BitBuf[0x0C97]
#define D8152   plc_16BitBuf[0x0C98]
#define D8153   plc_16BitBuf[0x0C99]
#define D8154   plc_16BitBuf[0x0C9A]
#define D8155   plc_16BitBuf[0x0C9B]
#define D8156   plc_16BitBuf[0x0C9C]
#define D8157   plc_16BitBuf[0x0C9F]
#define D8158   plc_16BitBuf[0x0C9E]
#define D8159   plc_16BitBuf[0x0C9F]
#define D8160   plc_16BitBuf[0x0CA0]
#define D8161   plc_16BitBuf[0x0CA1]
#define D8162   plc_16BitBuf[0x0CA2]
#define D8163   plc_16BitBuf[0x0CA3]
#define D8164   plc_16BitBuf[0x0CA4]
#define D8165   plc_16BitBuf[0x0CA5]
#define D8166   plc_16BitBuf[0x0CA6]
#define D8167   plc_16BitBuf[0x0CA7]
#define D8168   plc_16BitBuf[0x0CA8]
#define D8169   plc_16BitBuf[0x0CA9]
#define D8170   plc_16BitBuf[0x0CAA]
#define D8171   plc_16BitBuf[0x0CAB]
#define D8172   plc_16BitBuf[0x0CAC]
#define D8173   plc_16BitBuf[0x0CAD]
#define D8174   plc_16BitBuf[0x0CAE]
#define D8175   plc_16BitBuf[0x0CAF]
#define D8176   plc_16BitBuf[0x0CB0]
#define D8177   plc_16BitBuf[0x0CB1]
#define D8178   plc_16BitBuf[0x0CB2]
#define D8179   plc_16BitBuf[0x0CB3]
#define D8180   plc_16BitBuf[0x0CB4]
#define D8181   plc_16BitBuf[0x0CB5]
#define D8182   plc_16BitBuf[0x0CB6]
#define D8183   plc_16BitBuf[0x0CB7]
#define D8184   plc_16BitBuf[0x0CB8]
#define D8185   plc_16BitBuf[0x0CB9]
#define D8186   plc_16BitBuf[0x0CBA]
#define D8187   plc_16BitBuf[0x0CBB]
#define D8188   plc_16BitBuf[0x0CBC]
#define D8189   plc_16BitBuf[0x0CBD]
#define D8190   plc_16BitBuf[0x0CBE]
#define D8191   plc_16BitBuf[0x0CBF]
#define D8192   plc_16BitBuf[0x0CC0]
#define D8193   plc_16BitBuf[0x0CC1]
#define D8194   plc_16BitBuf[0x0CC2]
#define D8195   plc_16BitBuf[0x0CC3]
#define D8196   plc_16BitBuf[0x0CC4]
#define D8197   plc_16BitBuf[0x0CC5]
#define D8198   plc_16BitBuf[0x0CC6]
#define D8199   plc_16BitBuf[0x0CC7]
#define D8200   plc_16BitBuf[0x0CC8]
#define D8201   plc_16BitBuf[0x0CC9]
#define D8202   plc_16BitBuf[0x0CCA]
#define D8203   plc_16BitBuf[0x0CCB]
#define D8204   plc_16BitBuf[0x0CCC]
#define D8205   plc_16BitBuf[0x0CCD]
#define D8206   plc_16BitBuf[0x0CCE]
#define D8207   plc_16BitBuf[0x0CCF]
#define D8208   plc_16BitBuf[0x0CD0]
#define D8209   plc_16BitBuf[0x0CD1]
#define D8210   plc_16BitBuf[0x0CD2]
#define D8211   plc_16BitBuf[0x0CD3]
#define D8212   plc_16BitBuf[0x0CD4]
#define D8213   plc_16BitBuf[0x0CD5]
#define D8214   plc_16BitBuf[0x0CD6]
#define D8215   plc_16BitBuf[0x0CD7]
#define D8216   plc_16BitBuf[0x0CD8]
#define D8217   plc_16BitBuf[0x0CD9]
#define D8218   plc_16BitBuf[0x0CDA]
#define D8219   plc_16BitBuf[0x0CDB]
#define D8220   plc_16BitBuf[0x0CDC]
#define D8221   plc_16BitBuf[0x0CDD]
#define D8222   plc_16BitBuf[0x0CDE]
#define D8223   plc_16BitBuf[0x0CDF]
#define D8224   plc_16BitBuf[0x0CE0]
#define D8225   plc_16BitBuf[0x0CE1]
#define D8226   plc_16BitBuf[0x0CE2]
#define D8227   plc_16BitBuf[0x0CE3]
#define D8228   plc_16BitBuf[0x0CE4]
#define D8229   plc_16BitBuf[0x0CE5]
#define D8230   plc_16BitBuf[0x0CE6]
#define D8231   plc_16BitBuf[0x0CE7]
#define D8232   plc_16BitBuf[0x0CE8]
#define D8233   plc_16BitBuf[0x0CE9]
#define D8234   plc_16BitBuf[0x0CEA]
#define D8235   plc_16BitBuf[0x0CEB]
#define D8236   plc_16BitBuf[0x0CEC]
#define D8237   plc_16BitBuf[0x0CED]
#define D8238   plc_16BitBuf[0x0CEE]
#define D8239   plc_16BitBuf[0x0CEF]
#define D8240   plc_16BitBuf[0x0CF0]
#define D8241   plc_16BitBuf[0x0CF1]
#define D8242   plc_16BitBuf[0x0CF2]
#define D8243   plc_16BitBuf[0x0CF3]
#define D8244   plc_16BitBuf[0x0CF4]
#define D8245   plc_16BitBuf[0x0CF5]
#define D8246   plc_16BitBuf[0x0CF6]
#define D8247   plc_16BitBuf[0x0CF7]
#define D8248   plc_16BitBuf[0x0CF8]
#define D8249   plc_16BitBuf[0x0CF9]
#define D8250   plc_16BitBuf[0x0CFA]
#define D8251   plc_16BitBuf[0x0CFB]
#define D8252   plc_16BitBuf[0x0CFC]
#define D8253   plc_16BitBuf[0x0CFD]
#define D8254   plc_16BitBuf[0x0CFE]
#define D8255   plc_16BitBuf[0x0CFF]
/*************M8000-M8255****************/     
#define    	M8000   0X0F00+0
#define    	M8001   0X0F00+1
#define    	M8002	  0X0F00+2
#define    	M8003	  0X0F00+3
#define    	M8004	  0X0F00+4
#define    	M8005	  0X0F00+5
#define    	M8006	  0X0F00+6
#define    	M8007	  0X0F00+7
#define    	M8008	  0X0F00+8
#define    	M8009	  0X0F00+9
#define    	M8010	  0X0F00+10
#define    	M8011	  0X0F00+11
#define    	M8012	  0X0F00+12
#define    	M8013	  0X0F00+13
#define    	M8014	  0X0F00+14
#define    	M8015	  0X0F00+15
#define    	M8016	  0X0F00+16
#define    	M8017	  0X0F00+17
#define    	M8018	  0X0F00+18
#define    	M8019	  0X0F00+19
#define    	M8020	  0X0F00+20
#define    	M8021	  0X0F00+21
#define    	M8022	  0X0F00+22
#define    	M8023	  0X0F00+23
#define    	M8024	  0X0F00+24
#define    	M8025	  0X0F00+25
#define    	M8026	  0X0F00+26
#define    	M8027	  0X0F00+27
#define    	M8028	  0X0F00+28
#define    	M8029	  0X0F00+29
#define    	M8030	  0X0F00+30
#define    	M8031	  0X0F00+31
#define    	M8032	  0X0F00+32
#define    	M8033	  0X0F00+33
#define    	M8034	  0X0F00+34
#define    	M8035	  0X0F00+35
#define    	M8036	  0X0F00+36
#define    	M8037	  0X0F00+37
#define    	M8038	  0X0F00+38
#define    	M8039	  0X0F00+39
#define    	M8040	  0X0F00+40
#define    	M8041	  0X0F00+41
#define    	M8042	  0X0F00+42
#define    	M8043	  0X0F00+43
#define    	M8044	  0X0F00+44
#define    	M8045	  0X0F00+45
#define    	M8046	  0X0F00+46
#define    	M8047	  0X0F00+47
#define    	M8048	  0X0F00+48
#define    	M8049	  0X0F00+49
#define    	M8050	  0X0F00+50
#define    	M8051	  0X0F00+51
#define    	M8052	  0X0F00+52
#define    	M8053	  0X0F00+53
#define    	M8054	  0X0F00+54
#define    	M8055	  0X0F00+55
#define    	M8056	  0X0F00+56
#define    	M8057	  0X0F00+57
#define    	M8058	  0X0F00+58
#define    	M8059	  0X0F00+59
#define    	M8060	  0X0F00+60
#define    	M8061	  0X0F00+61
#define    	M8062	  0X0F00+62
#define    	M8063	  0X0F00+63
#define    	M8064	  0X0F00+64
#define    	M8065	  0X0F00+65
#define    	M8066	  0X0F00+66
#define    	M8067	  0X0F00+67
#define    	M8068	  0X0F00+68
#define    	M8069	  0X0F00+69
#define    	M8070	  0X0F00+70
#define    	M8071	  0X0F00+71
#define    	M8072	  0X0F00+72
#define    	M8073	  0X0F00+73
#define    	M8074	  0X0F00+74
#define    	M8075	  0X0F00+75
#define    	M8076	  0X0F00+76
#define    	M8077	  0X0F00+77
#define    	M8078	  0X0F00+78
#define    	M8079	  0X0F00+79
#define    	M8080	  0X0F00+80
#define    	M8081	  0X0F00+81
#define    	M8082	  0X0F00+82
#define    	M8083	  0X0F00+83
#define    	M8084	  0X0F00+84
#define    	M8085	  0X0F00+85
#define    	M8086	  0X0F00+86
#define    	M8087	  0X0F00+87
#define    	M8088	  0X0F00+88
#define    	M8089	  0X0F00+89
#define    	M8090	  0X0F00+90
#define    	M8091	  0X0F00+91
#define    	M8092	  0X0F00+92
#define    	M8093	  0X0F00+93
#define    	M8094	  0X0F00+94
#define    	M8095	  0X0F00+95
#define    	M8096	  0X0F00+96
#define    	M8097	  0X0F00+97
#define    	M8098	  0X0F00+98
#define    	M8099	  0X0F00+99
#define    	M8100	  0X0F00+100
#define    	M8101	  0X0F00+101
#define    	M8102	  0X0F00+102
#define    	M8103	  0X0F00+103
#define    	M8104	  0X0F00+104
#define    	M8105	  0X0F00+105
#define    	M8106	  0X0F00+106
#define    	M8107	  0X0F00+107
#define    	M8108	  0X0F00+108
#define    	M8109	  0X0F00+109
#define    	M8110	  0X0F00+110
#define    	M8111	  0X0F00+111
#define    	M8112	  0X0F00+112
#define    	M8113	  0X0F00+113
#define    	M8114	  0X0F00+114
#define    	M8115	  0X0F00+115
#define    	M8116	  0X0F00+116
#define    	M8117	  0X0F00+117
#define    	M8118	  0X0F00+118
#define    	M8119	  0X0F00+119
#define    	M8120	  0X0F00+120
#define    	M8121	  0X0F00+121
#define    	M8122	  0X0F00+122
#define    	M8123	  0X0F00+123
#define    	M8124	  0X0F00+124
#define    	M8125	  0X0F00+125
#define    	M8126	  0X0F00+126
#define    	M8127	  0X0F00+127
#define    	M8128	  0X0F00+128
#define    	M8129	  0X0F00+129
#define    	M8130	  0X0F00+130
#define    	M8131	  0X0F00+131
#define    	M8132	  0X0F00+132
#define    	M8133	  0X0F00+133
#define    	M8134	  0X0F00+134
#define    	M8135	  0X0F00+135
#define    	M8136	  0X0F00+136
#define    	M8137	  0X0F00+137
#define    	M8138	  0X0F00+138
#define    	M8139	  0X0F00+139
#define    	M8140	  0X0F00+140
#define    	M8141	  0X0F00+141
#define    	M8142	  0X0F00+142
#define    	M8143	  0X0F00+143
#define    	M8144	  0X0F00+144
#define    	M8145	  0X0F00+145
#define    	M8146	  0X0F00+146
#define    	M8147	  0X0F00+147
#define    	M8148	  0X0F00+148
#define    	M8149	  0X0F00+149
#define    	M8150	  0X0F00+150
#define    	M8151	  0X0F00+151
#define    	M8152	  0X0F00+152
#define    	M8153	  0X0F00+153
#define    	M8154	  0X0F00+154
#define    	M8155	  0X0F00+155
#define    	M8156	  0X0F00+156
#define    	M8157	  0X0F00+157
#define    	M8158	  0X0F00+158
#define    	M8159	  0X0F00+159
#define    	M8160	  0X0F00+160
#define    	M8161	  0X0F00+161
#define    	M8162	  0X0F00+162
#define    	M8163	  0X0F00+163
#define    	M8164	  0X0F00+164
#define    	M8165	  0X0F00+165
#define    	M8166	  0X0F00+166
#define    	M8167	  0X0F00+167
#define    	M8168	  0X0F00+168
#define    	M8169	  0X0F00+169
#define    	M8170	  0X0F00+170
#define    	M8171	  0X0F00+171
#define    	M8172	  0X0F00+172
#define    	M8173	  0X0F00+173
#define    	M8174	  0X0F00+174
#define    	M8175	  0X0F00+175
#define    	M8176	  0X0F00+176
#define    	M8177	  0X0F00+177
#define    	M8178	  0X0F00+178
#define    	M8179	  0X0F00+179
#define    	M8180	  0X0F00+180
#define    	M8181	  0X0F00+181
#define    	M8182	  0X0F00+182
#define    	M8183	  0X0F00+183
#define    	M8184	  0X0F00+184
#define    	M8185	  0X0F00+185
#define    	M8186	  0X0F00+186
#define    	M8187	  0X0F00+187
#define    	M8188	  0X0F00+188
#define    	M8189	  0X0F00+189
#define    	M8190	  0X0F00+190
#define    	M8191	  0X0F00+191
#define    	M8192	  0X0F00+192
#define    	M8193	  0X0F00+193
#define    	M8194	  0X0F00+194
#define    	M8195	  0X0F00+195
#define    	M8196	  0X0F00+196
#define    	M8197	  0X0F00+197
#define    	M8198	  0X0F00+198
#define    	M8199	  0X0F00+199
#define    	M8200	  0X0F00+200
#define    	M8201	  0X0F00+201
#define    	M8202	  0X0F00+202
#define    	M8203	  0X0F00+203
#define    	M8204	  0X0F00+204
#define    	M8205	  0X0F00+205
#define    	M8206	  0X0F00+206
#define    	M8207	  0X0F00+207
#define    	M8208	  0X0F00+208
#define    	M8209	  0X0F00+209
#define    	M8210	  0X0F00+210
#define    	M8211	  0X0F00+211
#define    	M8212	  0X0F00+212
#define    	M8213	  0X0F00+213
#define    	M8214	  0X0F00+214
#define    	M8215	  0X0F00+215
#define    	M8216	  0X0F00+216
#define    	M8217	  0X0F00+217
#define    	M8218	  0X0F00+218
#define    	M8219	  0X0F00+219
#define    	M8220	  0X0F00+220
#define    	M8221	  0X0F00+221
#define    	M8222	  0X0F00+222
#define    	M8223	  0X0F00+223
#define    	M8224	  0X0F00+224
#define    	M8225	  0X0F00+225
#define    	M8226	  0X0F00+226
#define    	M8227	  0X0F00+227
#define    	M8228	  0X0F00+228
#define    	M8229	  0X0F00+229
#define    	M8230	  0X0F00+230
#define    	M8231	  0X0F00+231
#define    	M8232	  0X0F00+232
#define    	M8233	  0X0F00+233
#define    	M8234	  0X0F00+234
#define    	M8235	  0X0F00+235
#define    	M8236	  0X0F00+236
#define    	M8237	  0X0F00+237
#define    	M8238	  0X0F00+238
#define    	M8239	  0X0F00+239
#define    	M8240	  0X0F00+240
#define    	M8241	  0X0F00+241
#define    	M8242	  0X0F00+242
#define    	M8243	  0X0F00+243
#define    	M8244	  0X0F00+244
#define    	M8245	  0X0F00+245
#define    	M8246	  0X0F00+246
#define    	M8247	  0X0F00+247
#define    	M8248	  0X0F00+248
#define    	M8249	  0X0F00+249
#define    	M8250	  0X0F00+250
#define    	M8251	  0X0F00+251
#define    	M8252	  0X0F00+252
#define    	M8253	  0X0F00+253
#define    	M8254	  0X0F00+254
#define    	M8255	  0X0F00+255


/////////////////////////////////////////////////////////
#define M8063_ERR_FLG				M8063  //串行出错
#define TIMEOUT_FLG   		  M8129  //超时的判断标志

//#define SET_COMM_PARM			  D8120  //通信格式的设定
#define SET_TIMEOUT    			D8129  //超时时间的设定
#define D8063_ERR_CODE      D8063  //串行通信出错1的错误代码编号

#define SET_RETRY_CNT    		D8132  //设定重试次数

#define MOD_TIMEOUT_DEF		  200   // 接收命令超时时间默认值, 单位ms
#define MOD_TIMEOUT_MAX		  3000   // 接收命令超时时间最大值, 单位ms
#define RETRY_CNT_DEF       6     // 重试次数默认值

/////////////////////////////////////////////////////////
#define HS_MOD_LOCAL_ADDR	  D8121  // 本机modbus通讯地址
#define HS_MOD_SAVLE_ADDR	  D8123  // 从站modbus通讯地址

/* 第1路modbus相关配置 */
#define HS_MOD1_MODE        D8120 // 第1路modbus主从机模式切换,1为主机模式,0为从机模式
//#define HS_MOD1_MODE	    1 // 第1路modbus主从机模式切换,1为主机模式,0为从机模式

// RS485芯片发送使能GPIO, PA3
#define HS_MOD1_RCC_RS485_EN 	 RCC_APB2Periph_GPIOE
#define HS_MOD1_PORT_RS485_EN  GPIOE
#define HS_MOD1_PIN_RS485_EN	 GPIO_Pin_15
#define HS_MOD1_RS485_RX_EN()	 HS_MOD1_PORT_RS485_EN->BRR = HS_MOD1_PIN_RS485_EN
#define HS_MOD1_RS485_TX_EN()	 HS_MOD1_PORT_RS485_EN->BSRR = HS_MOD1_PIN_RS485_EN
///////////////////////////////////////////////////////////////////////////////////

/* 第2路modbus相关配置 */
#define HS_MOD2_MODE	  D8122 // 第1路modbus主从机模式切换,1为主机模式,0为从机模式
//#define HS_MOD2_MODE	  0 // 第1路modbus主从机模式切换,1为主机模式,0为从机模式

// RS485芯片发送使能GPIO, PD0
#define HS_MOD2_RCC_RS485_EN 	 RCC_APB2Periph_GPIOB
#define HS_MOD2_PORT_RS485_EN  GPIOB
#define HS_MOD2_PIN_RS485_EN	 GPIO_Pin_1
#define HS_MOD2_RS485_RX_EN()	 HS_MOD2_PORT_RS485_EN->BRR = HS_MOD2_PIN_RS485_EN
#define HS_MOD2_RS485_TX_EN()	 HS_MOD2_PORT_RS485_EN->BSRR = HS_MOD2_PIN_RS485_EN

#define HS_MOD3_RCC_RS485_EN 	 RCC_APB2Periph_GPIOD
#define HS_MOD3_PORT_RS485_EN  GPIOD
#define HS_MOD3_PIN_RS485_EN	 GPIO_Pin_1
#define HS_MOD3_RS485_RX_EN()	 HS_MOD3_PORT_RS485_EN->BRR = HS_MOD3_PIN_RS485_EN
#define HS_MOD3_RS485_TX_EN()	 HS_MOD3_PORT_RS485_EN->BSRR = HS_MOD3_PIN_RS485_EN


#define HS_MOD3_MODE	  D8124 // 第3路modbus主从机模式切换,1为主机模式,0为从机模式

//////////////////////////////////////////////////////////////////////////////////

/* modbus公共部分 */

/* 01H 读强制单线圈、05H 写强制单线圈 、0FH 写多个线圈 */
/* S、Y、T、M、C、M8000、M1536*/

#define COIL_S_START_ADDR 				0
#define COIL_S_END_ADDR 		  		999

#define COIL_Y_START_ADDR 				1280
#define COIL_Y_END_ADDR 		  		1463

#define COIL_T_START_ADDR 				1536
#define COIL_T_END_ADDR 		  		1791

#define COIL_M_START_ADDR 				2048
#define COIL_M_END_ADDR 		  		3580

#define COIL_C_START_ADDR 				3584
#define COIL_C_END_ADDR 		  		3839

#define COIL_M8000_START_ADDR 		3840
#define COIL_M8000_END_ADDR 		  4095

#define COIL_M1538_START_ADDR 		10240
#define COIL_M1538_END_ADDR 		  11775

/* 02H 读取输入状态 */
#define INPUT_X_START_ADDR 				1024
#define INPUT_X_END_ADDR 		  		1215

/* 03H 读保持寄存器 */
/* 06H 写保持寄存器 */
/* 10H 写多个保存寄存器 */
/* C、C200、D8000、T、D*/
#define REG_C_START_ADDR 					1280
#define REG_C_END_ADDR 		  			1535

#define REG_C200_START_ADDR 		  1536
#define REG_C200_END_ADDR 		    1646

#define REG_D8000_START_ADDR 			1792
#define REG_D8000_END_ADDR 		  	2047

#define REG_T_START_ADDR 				  2048
#define REG_T_END_ADDR 		  		  2304

#define REG_D_START_ADDR 		      4096
#define REG_D_END_ADDR 		        12096

/* 04H 读取输入寄存器(模拟信号) */
// 暂略

#if W5500_FUNC
#define  MODBUS_NUM							4
#else
#define  MODBUS_NUM							3
#endif /* #if W5500_FUNC */

#define MOD_TX_RX_BUF_SIZE		  512

/* RTU 应答代码 */
#define RSP_OK				    0			// 成功 
#define RSP_ERR_CMD			  0x01	// 不支持的功能码
#define RSP_ERR_REG_ADDR	0x02	// 寄存器地址错误
#define RSP_ERR_VALUE		  0x03	// 数据值域错误
#define RSP_ERR_WRITE		  0x04	// 写入失败 

//////////////////////////////////////////////////////////////////////////////
#if RS_FUNC
//RS指令 殊寄存器和特殊继电器
//#define RS_SERIAL_ERR_FLG				M8063  //串行出错
#define RS_TX_WAIT_FLG			 	 	M8121  //发送等待标志
#define RS_TX_REQUEST_FLG				M8122  //发送请求
#define RS_RX_END_FLG       		M8123  //接收结束标志
#define RS_CARRIER_DETECT_FLG   M8124  //载波的检测标志
//#define RS_TIMEOUT_FLG   				M8129  //超时的判断标志
#define RS_8BIT_MODE_FLG   			M8161  //8位处理模式

//#define RS_SET_COMM_PARM			D8120  //通信格式的设定
#define RS_TX_REMAINDER_NUM			D8122  //发送数据的剩余点数
#define RS_RX_NUM         			D8123  //接收点数的监控
#define RS_HEADER_MSG         	D8124  //报头
#define RS_END_MSG			        D8125  //报尾
//#define RS_SET_TIMEOUT    			D8129  //超时时间的设定
//#define RS_SERIAL_ERR     			D8063  //串行通信出错1的错误代码编号
//#define RS_DISPLAY_COMM_PARM		D8405  //通信参数的显示
//#define RS_DISPLAY_RUN_MODE			D8419  //运行模式的显示
#endif /* #if RS_FUNC */
//////////////////////////////////////////////////////////////////////////////

/* 传人记,20161124新增,modbus功能相关配置 */
extern uint8_t md_config[3][2];

#endif   
/*********************************************************************************************************
**                            End Of File
********************************************************************************************************/
