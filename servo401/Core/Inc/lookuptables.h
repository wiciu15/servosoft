/*
 * lookuptables.h
 *
 *  Created on: Feb 6, 2022
 *      Author: Wiktor
 */

#ifndef INC_LOOKUPTABLES_H_
#define INC_LOOKUPTABLES_H_


static const float sine_table [] = { //0-90 deg sine wave 1 deg increment
		0
		,0.0174524
		,0.0348994
		,0.0523359
		,0.0697564
		,0.0871557
		,0.1045284
		,0.1218693
		,0.1391731
		,0.1564344
		,0.1736481
		,0.1908089
		,0.2079116
		,0.2249510
		,0.2419218
		,0.2588190
		,0.2756373
		,0.2923717
		,0.3090169
		,0.3255681
		,0.3420201
		,0.3583679
		,0.3746065
		,0.3907311
		,0.4067366
		,0.4226182
		,0.4383711
		,0.4539905
		,0.4694715
		,0.4848096
		,0.5
		,0.5150380
		,0.5299192
		,0.5446390
		,0.5591929
		,0.5735764
		,0.5877852
		,0.6018150
		,0.6156614
		,0.6293203
		,0.6427876
		,0.6560590
		,0.6691306
		,0.6819983
		,0.6946583
		,0.7071067
		,0.7193398
		,0.7313537
		,0.7431448
		,0.7547095
		,0.7660444
		,0.7771459
		,0.7880107
		,0.7986355
		,0.8090169
		,0.8191520
		,0.8290375
		,0.8386705
		,0.8480480
		,0.8571673
		,0.8660254
		,0.8746197
		,0.8829475
		,0.8910065
		,0.8987940
		,0.9063077
		,0.9135454
		,0.9205048
		,0.9271838
		,0.9335804
		,0.9396926
		,0.9455185
		,0.9510565
		,0.9563047
		,0.9612616
		,0.9659258
		,0.9702957
		,0.9743700
		,0.9781476
		,0.9816271
		,0.9848077
		,0.9876883
		,0.9902680
		,0.9925461
		,0.9945218
		,0.9961946
		,0.9975640
		,0.9986295
		,0.9993908
		,0.9998476
		,1
};
static const float  t1calculated [] =
       {0.866025404,
		0.857167301,
		0.848048096,
		0.838670568,
		0.829037573,
		0.819152044,
		0.809016994,
		0.79863551,
		0.788010754,
		0.777145961,
		0.766044443,
		0.75470958,
		0.743144825,
		0.731353702,
		0.7193398,
		0.707106781,
		0.69465837,
		0.68199836,
		0.669130606,
		0.656059029,
		0.64278761,
		0.629320391,
		0.615661475,
		0.601815023,
		0.587785252,
		0.573576436,
		0.559192903,
		0.544639035,
		0.529919264,
		0.515038075,
		0.5,
		0.48480962,
		0.469471563,
		0.4539905,
		0.438371147,
		0.422618262,
		0.406736643,
		0.390731128,
		0.374606593,
		0.35836795,
		0.342020143,
		0.325568154,
		0.309016994,
		0.292371705,
		0.275637356,
		0.258819045,
		0.241921896,
		0.224951054,
		0.207911691,
		0.190808995,
		0.173648178,
		0.156434465,
		0.139173101,
		0.121869343,
		0.104528463,
		0.087155743,
		0.069756474,
		0.052335956,
		0.034899497,
		0.017452406,
		0
};

static const float t2calculated []=
{		0,
		0.020152303,
		0.040298468,
		0.060432357,
		0.080547838,
		0.100638783,
		0.120699073,
		0.140722596,
		0.160703255,
		0.180634961,
		0.200511644,
		0.22032725,
		0.240075741,
		0.259751104,
		0.279347343,
		0.298858491,
		0.318278603,
		0.337601765,
		0.35682209,
		0.375933723,
		0.394930844,
		0.413807664,
		0.432558435,
		0.451177444,
		0.469659021,
		0.487997534,
		0.506187399,
		0.524223074,
		0.542099066,
		0.55980993,
		0.577350269,
		0.594714742,
		0.61189806,
		0.628894987,
		0.645700347,
		0.66230902,
		0.678715947,
		0.694916131,
		0.710904637,
		0.726676594,
		0.742227199,
		0.757551714,
		0.772645471,
		0.787503874,
		0.802122394,
		0.816496581,
		0.830622055,
		0.844494513,
		0.85810973,
		0.871463559,
		0.884551931,
		0.89737086,
		0.909916441,
		0.922184853,
		0.934172359,
		0.945875307,
		0.957290131,
		0.968413356,
		0.979241593,
		0.989771544,
		1
};

#endif /* INC_LOOKUPTABLES_H_ */