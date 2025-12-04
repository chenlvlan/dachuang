#include <Arduino.h>
#include <ZE300_485.h>

enum
{
	RS485_RX_PIN = 16,
	RS485_TX_PIN = 17,
	RS485_TNOW_PIN = 18,
	minAngle = 10,
	maxAngle = 90,

};

ZE300 joint;
information_t jointMoterInfo[4];

void setup()
{
	Serial.begin(115200);
	joint.init(921600, RS485_RX_PIN, RS485_TX_PIN, RS485_TNOW_PIN);
	joint.isDebug = 0;
	// delay(2000);
	joint.refreshAllInformation(11, &jointMoterInfo[0], 1);
	joint.refreshAllInformation(12, &jointMoterInfo[1], 1);
	joint.refreshAllInformation(13, &jointMoterInfo[2], 1);
	joint.refreshAllInformation(14, &jointMoterInfo[3], 1);
	delay(2000);

#if 0
  joint.multiCloseLoopControl(3, 40, 1, &jointMoterInfo[0]);
  joint.multiCloseLoopControl(4, 40, 1, &jointMoterInfo[1]);
  joint.absolutePosControl(3, minAngle, &jointMoterInfo[0]);
  joint.absolutePosControl(4, minAngle, &jointMoterInfo[1]);

  delay(5000);

  joint.multiCloseLoopControl(3, 960, 5, &jointMoterInfo[0]);
  joint.multiCloseLoopControl(4, 960, 5, &jointMoterInfo[1]);
  joint.absolutePosControl(3, maxAngle, &jointMoterInfo[0]);
  joint.absolutePosControl(4, maxAngle, &jointMoterInfo[1]);
  delay(200);
  joint.multiCloseLoopControl(3, 160, 0.75, &jointMoterInfo[0]);
  joint.multiCloseLoopControl(4, 160, 0.75, &jointMoterInfo[1]);
  joint.absolutePosControl(3, 40, &jointMoterInfo[0]);
  joint.absolutePosControl(4, 40, &jointMoterInfo[1]);
#endif
}

void loop()
{
	// #if 0
	while (Serial.available() > 0)
	{
		Serial.read();
		joint.getRealTimeData(11, &jointMoterInfo[0], 1);
	}

	// delay(200);
	// #endif

	for (int i = 11; i <= 14; i++)
	{
		joint.multiCloseLoopControl(i, 40, 1, &jointMoterInfo[0]);
		joint.absolutePosControl(i, 0, &jointMoterInfo[0]);
	}
	delay(2000);
	for (int i = 11; i <= 14; i++)
	{
		joint.multiCloseLoopControl(i, 40, 1, &jointMoterInfo[0]);
		joint.absolutePosControl(i, 30, &jointMoterInfo[0]);
	}
	delay(2000);
	// Serial.println(analogReadMilliVolts(27));
}