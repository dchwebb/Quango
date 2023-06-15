#include "SerialHandler.h"
#include "Calib.h"

#include <stdio.h>
#include <cmath>		// for cordic test

int32_t SerialHandler::ParseInt(const std::string cmd, const char precedingChar, int low = 0, int high = 0) {
	int32_t val = -1;
	int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(cmd.substr(pos + 1).c_str(), "0123456789-") > 0) {
		val = stoi(cmd.substr(pos + 1));
	}
	if (high > low && (val > high || val < low)) {
		usb->SendString("Must be a value between " + std::to_string(low) + " and " + std::to_string(high) + "\r\n");
		return low - 1;
	}
	return val;
}

float SerialHandler::ParseFloat(const std::string cmd, const char precedingChar, float low = 0.0, float high = 0.0) {
	float val = -1.0f;
	int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(cmd.substr(pos + 1).c_str(), "0123456789.") > 0) {
		val = stof(cmd.substr(pos + 1));
	}
	if (high > low && (val > high || val < low)) {
		usb->SendString("Must be a value between " + std::to_string(low) + " and " + std::to_string(high) + "\r\n");
		return low - 1.0f;
	}
	return val;
}

SerialHandler::SerialHandler(USB& usbObj)
{
	usb = &usbObj;

	// bind the usb's CDC caller to the CDC handler in this class
	usb->cdcDataHandler = std::bind(&SerialHandler::Handler, this, std::placeholders::_1, std::placeholders::_2);
}



// Check if a command has been received from USB, parse and action as required
bool SerialHandler::Command()
{
	if (!CmdPending) {
		return false;
	}

	if (ComCmd.compare("info\n") == 0) {		// Print diagnostic information

		usb->SendString("Mountjoy Quango v1.0 - Current Settings:\r\n\r\n");

	} else if (ComCmd.compare("help\n") == 0) {

		usb->SendString("Mountjoy Quango\r\n"
				"\r\nSupported commands:\r\n"
				"info        -  Show diagnostic information\r\n"
				"calib       -  Show calibration settings\r\n"
				"lfo         -  Tremolo LFO on/off\r\n"
				"\r\n"
#if (USB_DEBUG)
				"usbdebug    -  Start USB debugging\r\n"
				"\r\n"
#endif
		);

#if (USB_DEBUG)
	} else if (ComCmd.compare("usbdebug\n") == 0) {				// Configure gate LED
		USBDebug = true;
		usb->SendString("Press link button to dump output\r\n");
#endif

	} else if (ComCmd.compare("calib\n") == 0) {				// Print calibration settings
		auto buffPos = buf;
		buffPos += sprintf(buffPos, "\r\nCalibration Settings: ");
		for (uint8_t chn = 0; chn < 2; ++chn) {
			buffPos += sprintf(buffPos, "\r\nChannel %c", chn ? 'B' : 'A');
			for (uint8_t v = 0; v < 4; ++v) {
				buffPos += sprintf(buffPos, "\r\nVoice %d\r\n", v + 1);
				for (uint8_t octave = 0; octave < 6; ++octave) {
					buffPos += sprintf(buffPos, "%f  ", calib.calibOffsets[chn][v][octave]);
				}
			}
		}
		sprintf(buffPos, "\r\n\0");
		usb->SendString(buf);


	} else if (ComCmd.compare("l\n") == 0) {				// Long envelope times
		//envelope.longTimes = true;

	} else if (ComCmd.compare("s\n") == 0) {				// Short envelope times
		//envelope.longTimes = false;


	} else {
		usb->SendString("Unrecognised command: " + ComCmd + "Type 'help' for supported commands\r\n");
	}

	CmdPending = false;
	return true;
}


void SerialHandler::Handler(uint8_t* data, uint32_t length)
{
	static bool newCmd = true;
	if (newCmd) {
		ComCmd = std::string(reinterpret_cast<char*>(data), length);
		newCmd = false;
	} else {
		ComCmd.append(reinterpret_cast<char*>(data), length);
	}
	if (*ComCmd.rbegin() == '\r')
		*ComCmd.rbegin() = '\n';

	if (*ComCmd.rbegin() == '\n') {
		CmdPending = true;
		newCmd = true;
	}

}

