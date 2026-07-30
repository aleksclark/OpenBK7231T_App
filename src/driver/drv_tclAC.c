// https://github.com/lNikazzzl/tcl_ac_esphome/tree/master

#include "../obk_config.h"

#if ENABLE_DRIVER_TCL

#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../httpserver/new_http.h"
#include "../hal/hal_flashVars.h"
#include "drv_uart.h"

#define TCL_UART_PACKET_LEN 1
#define TCL_UART_PACKET_HEAD 0xff
#define TCL_UART_RECEIVE_BUFFER_SIZE 256
#define TCL_baudRate	9600

#include "drv_tclAC.h"

uint8_t set_cmd_base[35] = { 0xBB, 0x00, 0x01, 0x03, 0x1D, 0x00, 0x00, 0x64, 0x03, 0xF3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
bool ready_to_send_set_cmd_flag = false;
set_cmd_t m_set_cmd = { 0 };
get_cmd_resp_t m_get_cmd_resp = { 0 };
int g_buzzer = 1;
int g_disp = 1;
int g_gen = 0;

// Dual-setpoint (heat_cool) state
bool g_heat_cool_mode = false;
uint8_t g_heat_cool_last_action = 0;  // 0x01=cool, 0x04=heat, 0x02=idle(fan)
float g_heat_cool_low = 18.0f;   // heat-to target (°C)
float g_heat_cool_high = 25.0f;  // cool-to target (°C)

// Staggered publish: instead of publishing all values at once (which overflows
// the RTL87x0C lwIP TCP send buffer causing ERR_MEM -> forced reconnect every ~5min),
// we publish ONE value per second in rotation. Each value still updates every 10s.
// Change-based publishes still fire immediately for responsiveness.
#define TCL_PUBLISH_SLOT_COUNT 10
static int tcl_publish_slot = 0;  // current rotation slot (0..9)
static int tcl_prev_current_temp = -999;
static int tcl_prev_target_low = -999;
static int tcl_prev_target_high = -999;
static int tcl_prev_mode = -1;
static int tcl_prev_fan = -1;
static int tcl_prev_buzzer = -1;
static int tcl_prev_disp = -1;
static int tcl_prev_swingH = -1;
static int tcl_prev_swingV = -1;
static const char* tcl_prev_action = "";

// Flash persistence channels for heat_cool state (survives reboots)
#define TCL_FLASH_CH_HEAT_COOL_LOW   10
#define TCL_FLASH_CH_HEAT_COOL_HIGH  11
#define TCL_FLASH_CH_HEAT_COOL_MODE  12

typedef enum {
	CLIMATE_MODE_OFF,
	CLIMATE_MODE_COOL,
	CLIMATE_MODE_DRY,
	CLIMATE_MODE_FAN_ONLY,
	CLIMATE_MODE_HEAT,
	CLIMATE_MODE_HEAT_COOL,
	CLIMATE_MODE_AUTO,
} climateMode_e;

// Forward declarations for globals defined later in this file
// (needed because OBK_SetClimate and TCL_ApplyHeatCoolLogic reference them)
climateMode_e g_mode = CLIMATE_MODE_OFF;
float current_temperature;

void build_set_cmd(get_cmd_resp_t * get_cmd_resp) {
	memcpy(m_set_cmd.raw, set_cmd_base, sizeof(m_set_cmd.raw));


	ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "build_set_cmd: sizeof(get_cmd_resp_t) = %i, sizeof(m_set_cmd.data) = %i, sizeof(m_set_cmd.raw) = %i", 
		sizeof(get_cmd_resp_t), sizeof(m_set_cmd.data), sizeof(m_set_cmd.raw));
	m_set_cmd.data.power = get_cmd_resp->data.power;
	m_set_cmd.data.off_timer_en = 0;
	m_set_cmd.data.on_timer_en = 0;
	m_set_cmd.data.beep = g_buzzer;
	m_set_cmd.data.disp = g_disp;
	m_set_cmd.data.eco = 0;

	switch (get_cmd_resp->data.mode) {
	case 0x01:
		m_set_cmd.data.mode = 0x03;
		break;
	case 0x03:
		m_set_cmd.data.mode = 0x02;
		break;
	case 0x02:
		m_set_cmd.data.mode = 0x07;
		break;
	case 0x04:
		m_set_cmd.data.mode = 0x01;
		break;
	case 0x05:
		m_set_cmd.data.mode = 0x08;
		break;
	}

	m_set_cmd.data.turbo = get_cmd_resp->data.turbo;
	m_set_cmd.data.mute = get_cmd_resp->data.mute;
	m_set_cmd.data.temp = 15 - get_cmd_resp->data.temp;

	switch (get_cmd_resp->data.fan) {
	case 0x00:
		m_set_cmd.data.fan = 0x00;
		break;
	case 0x01:
		m_set_cmd.data.fan = 0x02;
		break;
	case 0x04:
		m_set_cmd.data.fan = 0x06;
		break;
	case 0x02:
		m_set_cmd.data.fan = 0x03;
		break;
	case 0x05:
		m_set_cmd.data.fan = 0x07;
		break;
	case 0x03:
		m_set_cmd.data.fan = 0x05;
		break;
	}

	//m_set_cmd.data.vswing = get_cmd_resp->data.vswing ? 0x07 : 0x00;
	//m_set_cmd.data.hswing = get_cmd_resp->data.hswing;

	if (get_cmd_resp->data.vswing_mv) {
		m_set_cmd.data.vswing = 0x07;
		m_set_cmd.data.vswing_fix = 0;
		m_set_cmd.data.vswing_mv = get_cmd_resp->data.vswing_mv;
	}
	else if (get_cmd_resp->data.vswing_fix) {
		m_set_cmd.data.vswing = 0;
		m_set_cmd.data.vswing_fix = get_cmd_resp->data.vswing_fix;
		m_set_cmd.data.vswing_mv = 0;
	}

	if (get_cmd_resp->data.hswing_mv) {
		m_set_cmd.data.hswing = 0x01;
		m_set_cmd.data.hswing_fix = 0;
		m_set_cmd.data.hswing_mv = get_cmd_resp->data.hswing_mv;
	}
	else if (get_cmd_resp->data.hswing_fix) {
		m_set_cmd.data.hswing = 0;
		m_set_cmd.data.hswing_fix = get_cmd_resp->data.hswing_fix;
		m_set_cmd.data.hswing_mv = 0;
	}

	m_set_cmd.data.half_degree = 0;
	m_set_cmd.data.byte_7_bit_0_1 = g_gen;

	for (int i = 0; i < sizeof(m_set_cmd.raw) - 1; i++) m_set_cmd.raw[sizeof(m_set_cmd.raw) - 1] ^= m_set_cmd.raw[i];
}
typedef enum {
	FAN_OFF,
	FAN_1, // 1
	FAN_2, // 2
	FAN_3, // 3
	FAN_4, // 4
	FAN_5, // 5

	FAN_MUTE, // 6
	FAN_TURBO,
	FAN_AUTOMATIC,

} fanMode_e;

static const struct {
	const char *name;
	fanMode_e mode;
} fanModeMap[] = {
	{"off", FAN_OFF},
	{"1", FAN_1},
	{"2", FAN_2},
	{"3", FAN_3},
	{"4", FAN_4},
	{"5", FAN_5},
	{"mute", FAN_MUTE},
	{"turbo", FAN_TURBO},
	{"auto", FAN_AUTOMATIC},
};
//const char *fanOptions[] = { "auto", "low", "medium", "high" };
const char *fanOptions[] = { "off", "1", "2", "3", "4", "5", "mute", "turbo", "auto" };

fanMode_e parseFanMode(const char *s) {
	for (int i = 0; i < sizeof(fanModeMap) / sizeof(fanModeMap[0]); ++i) {
		if (!stricmp(s, fanModeMap[i].name)) {
			return fanModeMap[i].mode;
		}
	}
	return (fanMode_e)atoi(s);
}

const char *fanModeToStr(fanMode_e mode) {
	for (int i = 0; i < sizeof(fanModeMap) / sizeof(fanModeMap[0]); ++i) {
		if (fanModeMap[i].mode == mode) {
			return fanModeMap[i].name;
		}
	}
	return NULL;
}
typedef enum {
	VS_NONE,
	VS_MoveFull,
	VS_MoveUpper,
	VS_MoveLower,
	VS_FixTop,
	VS_FixUpper,
	VS_FixMid,
	VS_FixLower,
	VS_FixBottom
} VerticalSwingMode;
typedef enum {
	HS_NONE,
	HS_MOVE_FULL,
	HS_MOVE_LEFT,
	HS_MOVE_MID,
	HS_MOVE_RIGHT,
	HS_FIX_LEFT,
	HS_FIX_MID_LEFT,
	HS_FIX_MID,
	HS_FIX_MID_RIGHT,
	HS_FIX_RIGHT
} HorizontalSwing;
const char* vertical_swing_options[] = {
	"none",
	"move_full",
	"move_upper",
	"move_lower",
	"fix_top",
	"fix_upper",
	"fix_mid",
	"fix_lower",
	"fix_bottom"
};

const char* horizontal_swing_options[] = {
	"none",
	"move_full",
	"move_left",
	"move_mid",
	"move_right",
	"fix_left",
	"fix_mid_left",
	"fix_mid",
	"fix_mid_right",
	"fix_right"
};
const char *getSwingVLabel(VerticalSwingMode m) {
	return vertical_swing_options[m];
}
const char *getSwingHLabel(HorizontalSwing m) {
	return horizontal_swing_options[m];
}
VerticalSwingMode parse_vertical_swing(const char *s) {
	for (int i = 0; i < sizeof(vertical_swing_options) / sizeof(vertical_swing_options[0]); ++i) {
		if (stricmp(s, vertical_swing_options[i]) == 0)
			return (VerticalSwingMode)i;
	}
	return atoi(s);
}

HorizontalSwing parse_horizontal_swing(const char *s) {
	for (int i = 0; i < sizeof(horizontal_swing_options) / sizeof(horizontal_swing_options[0]); ++i) {
		if (stricmp(s, horizontal_swing_options[i]) == 0)
			return (HorizontalSwing)i;
	}
	return atoi(s);
}
void OBK_SetTargetTemperature(float temp) {
	// User requested target temperature change

	get_cmd_resp_t get_cmd_resp = { 0 };
	memcpy(get_cmd_resp.raw, m_get_cmd_resp.raw, sizeof(get_cmd_resp.raw));

	get_cmd_resp.data.temp = (uint8_t)(temp) - 16;

	build_set_cmd(&get_cmd_resp);
	ready_to_send_set_cmd_flag = true;
}
void OBK_SetFanMode(fanMode_e fan_mode) {

	get_cmd_resp_t get_cmd_resp = { 0 };
	memcpy(get_cmd_resp.raw, m_get_cmd_resp.raw, sizeof(get_cmd_resp.raw));

	get_cmd_resp.data.turbo = 0x00;
	get_cmd_resp.data.mute = 0x00;
	if (fan_mode == FAN_TURBO) {
		get_cmd_resp.data.fan = 0x03;
		get_cmd_resp.data.turbo = 0x01;
	}
	else if (fan_mode == FAN_MUTE) {
		get_cmd_resp.data.fan = 0x01;
		get_cmd_resp.data.mute = 0x01;
	}
	else if (fan_mode == FAN_AUTOMATIC) get_cmd_resp.data.fan = 0x00;
	else if (fan_mode == FAN_1) get_cmd_resp.data.fan = 0x01;
	else if (fan_mode == FAN_2) get_cmd_resp.data.fan = 0x04;
	else if (fan_mode == FAN_3) get_cmd_resp.data.fan = 0x02;
	else if (fan_mode == FAN_4) get_cmd_resp.data.fan = 0x05;
	else if (fan_mode == FAN_5) get_cmd_resp.data.fan = 0x03;

	build_set_cmd(&get_cmd_resp);
	ready_to_send_set_cmd_flag = true;

}
void OBK_SetGen(int gen) {

	get_cmd_resp_t get_cmd_resp = { 0 };
	memcpy(get_cmd_resp.raw, m_get_cmd_resp.raw, sizeof(get_cmd_resp.raw));

	g_gen = gen;

	build_set_cmd(&get_cmd_resp);
	ready_to_send_set_cmd_flag = true;
}
void OBK_SetBuzzer(int buzzer) {

	get_cmd_resp_t get_cmd_resp = { 0 };
	memcpy(get_cmd_resp.raw, m_get_cmd_resp.raw, sizeof(get_cmd_resp.raw));

	g_buzzer = buzzer;

	build_set_cmd(&get_cmd_resp);
	ready_to_send_set_cmd_flag = true;
}
void OBK_SetDisplay(int display) {

	get_cmd_resp_t get_cmd_resp = { 0 };
	memcpy(get_cmd_resp.raw, m_get_cmd_resp.raw, sizeof(get_cmd_resp.raw));

	g_disp = display;

	build_set_cmd(&get_cmd_resp);
	ready_to_send_set_cmd_flag = true;
}
void TCL_ApplyHeatCoolLogic(void);
void OBK_SetClimate(climateMode_e climate_mode)
{
	ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "User set mode %i", climate_mode);

	if (climate_mode == CLIMATE_MODE_OFF) {
		g_heat_cool_mode = false;
		HAL_FlashVars_SaveChannel(TCL_FLASH_CH_HEAT_COOL_MODE, 0);
		get_cmd_resp_t get_cmd_resp = { 0 };
		memcpy(get_cmd_resp.raw, m_get_cmd_resp.raw, sizeof(get_cmd_resp.raw));
		get_cmd_resp.data.power = 0x00;
		build_set_cmd(&get_cmd_resp);
		ready_to_send_set_cmd_flag = true;
		return;
	}

	if (climate_mode == CLIMATE_MODE_FAN_ONLY) {
		g_heat_cool_mode = false;
		HAL_FlashVars_SaveChannel(TCL_FLASH_CH_HEAT_COOL_MODE, 0);
		get_cmd_resp_t get_cmd_resp = { 0 };
		memcpy(get_cmd_resp.raw, m_get_cmd_resp.raw, sizeof(get_cmd_resp.raw));
		get_cmd_resp.data.power = 0x01;
		get_cmd_resp.data.mode = 0x02;  // fan_only
		build_set_cmd(&get_cmd_resp);
		ready_to_send_set_cmd_flag = true;
		return;
	}

	// Everything else (including heat_cool) enters dual-setpoint mode
	g_heat_cool_mode = true;
	HAL_FlashVars_SaveChannel(TCL_FLASH_CH_HEAT_COOL_MODE, 1);
	g_mode = CLIMATE_MODE_HEAT_COOL;
	TCL_ApplyHeatCoolLogic();
}
void write_array(byte *b, int s) {
	for (int i = 0; i < s; i++) {
		UART_SendByte(b[i]);
	}
}


static const struct {
	const char *name;
	climateMode_e mode;
} climateModeMap[] = {
	{"off", CLIMATE_MODE_OFF},
	{"fan_only", CLIMATE_MODE_FAN_ONLY},
	{"heat_cool", CLIMATE_MODE_HEAT_COOL},
};

climateMode_e parseClimate(const char *s) {
	for (int i = 0; i < sizeof(climateModeMap) / sizeof(climateModeMap[0]); ++i) {
		if (!stricmp(s, climateModeMap[i].name)) {
			return climateModeMap[i].mode;
		}
	}
	return (climateMode_e)atoi(s);
}

const char *climateModeToStr(climateMode_e mode) {
	for (int i = 0; i < sizeof(climateModeMap) / sizeof(climateModeMap[0]); ++i) {
		if (climateModeMap[i].mode == mode) {
			return climateModeMap[i].name;
		}
	}
	return NULL;
}

int read_data_line(int readch, uint8_t *buffer, int len)
{
	static int pos = 0;
	static bool wait_len = false;
	static int skipch = 0;

	//ESP_LOGI("custom", "%02X", readch);

	if (readch >= 0) {
		if (readch == 0xBB && skipch == 0 && !wait_len) {
			pos = 0;
			skipch = 3; // wait char with len
			wait_len = true;
			if (pos < len - 1) buffer[pos++] = readch;
		}
		else if (skipch == 0 && wait_len) {
			if (pos < len - 1) buffer[pos++] = readch;
			skipch = readch + 1; // +1 control sum
			wait_len = false;
		}
		else if (skipch > 0) {
			if (pos < len - 1) buffer[pos++] = readch;
			if (--skipch == 0 && !wait_len) return pos;
		}
	}
	// No end of line has been found, so return -1.
	return -1;
}

bool is_valid_xor(uint8_t *buffer, int len)
{
	uint8_t xor_byte = 0;
	for (int i = 0; i < len - 1; i++) xor_byte ^= buffer[i];
	if (xor_byte == buffer[len - 1]) return true;
	else {
		ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "No valid xor crc %02X (calculated %02X)", buffer[len], xor_byte);
		return false;
	}

}
void control_vertical_swing(VerticalSwingMode swing_mode) {
	get_cmd_resp_t get_cmd_resp = { 0 };
	memcpy(get_cmd_resp.raw, m_get_cmd_resp.raw, sizeof(get_cmd_resp.raw));

	get_cmd_resp.data.vswing_mv = 0;
	get_cmd_resp.data.vswing_fix = 0;

	switch (swing_mode) {
	case VS_MoveFull:   get_cmd_resp.data.vswing_mv = 0x01; break;
	case VS_MoveUpper:  get_cmd_resp.data.vswing_mv = 0x02; break;
	case VS_MoveLower:  get_cmd_resp.data.vswing_mv = 0x03; break;
	case VS_FixTop:     get_cmd_resp.data.vswing_fix = 0x01; break;
	case VS_FixUpper:   get_cmd_resp.data.vswing_fix = 0x02; break;
	case VS_FixMid:     get_cmd_resp.data.vswing_fix = 0x03; break;
	case VS_FixLower:   get_cmd_resp.data.vswing_fix = 0x04; break;
	case VS_FixBottom:  get_cmd_resp.data.vswing_fix = 0x05; break;
	case VS_NONE: default:  break;
	}

	get_cmd_resp.data.vswing = (get_cmd_resp.data.vswing_mv != 0) ? 0x01 : 0;

	build_set_cmd(&get_cmd_resp);
	ready_to_send_set_cmd_flag = true;
}
void control_horizontal_swing(HorizontalSwing swing_mode) {
	get_cmd_resp_t get_cmd_resp = { 0 };
	memcpy(get_cmd_resp.raw, m_get_cmd_resp.raw, sizeof(get_cmd_resp.raw));

	get_cmd_resp.data.hswing_mv = 0;
	get_cmd_resp.data.hswing_fix = 0;

	switch (swing_mode) {
	case HS_MOVE_FULL:      get_cmd_resp.data.hswing_mv = 0x01; break;
	case HS_MOVE_LEFT:      get_cmd_resp.data.hswing_mv = 0x02; break;
	case HS_MOVE_MID:       get_cmd_resp.data.hswing_mv = 0x03; break;
	case HS_MOVE_RIGHT:     get_cmd_resp.data.hswing_mv = 0x04; break;
	case HS_FIX_LEFT:       get_cmd_resp.data.hswing_fix = 0x01; break;
	case HS_FIX_MID_LEFT:   get_cmd_resp.data.hswing_fix = 0x02; break;
	case HS_FIX_MID:        get_cmd_resp.data.hswing_fix = 0x03; break;
	case HS_FIX_MID_RIGHT:  get_cmd_resp.data.hswing_fix = 0x04; break;
	case HS_FIX_RIGHT:      get_cmd_resp.data.hswing_fix = 0x05; break;
	case HS_NONE: default:  break;
	}

	if (get_cmd_resp.data.vswing_mv) get_cmd_resp.data.hswing = 0x01;
	else get_cmd_resp.data.hswing = 0;

	build_set_cmd(&get_cmd_resp);
	ready_to_send_set_cmd_flag = true;
}

void print_hex_str(uint8_t *buffer, int len)
{
	char str[250] = { 0 };
	char *pstr = str;
	if (len * 2 > sizeof(str)) ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "too long byte data");

	for (int i = 0; i < len; i++) {
		pstr += sprintf(pstr, "%02X ", buffer[i]);
	}

	ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "%s", str);
}
bool is_changed;
float target_temperature;
void set_target_temperature(float newTemp) {
	if (target_temperature == newTemp)
		return;
	is_changed = true;
	target_temperature = newTemp;
}
fanMode_e g_fanMode;
void set_mode(climateMode_e mode) {
	if (g_mode == mode)
		return;
	is_changed = true;
	g_mode = mode;
}
VerticalSwingMode g_swingV;
HorizontalSwing g_swingH;
void set_swingV(VerticalSwingMode mode) {
	if (g_swingV == mode)
		return;
	is_changed = true;
	g_swingV = mode;
}
void set_swingH(HorizontalSwing mode) {
	if (g_swingH == mode)
		return;
	is_changed = true;
	g_swingH = mode;
}
void set_custom_fan_mode(fanMode_e mode) {
	if (g_fanMode == mode)
		return;
	is_changed = true;
	g_fanMode = mode;
}
void set_current_temperature(float newTemp) {
	if (current_temperature == newTemp)
		return;
	is_changed = true;
	current_temperature = newTemp;
}
void TCL_UART_TryToGetNextPacket() {

	#define max_line_length 100
	static uint8_t buffer[max_line_length];

	ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "Initial size: %i", UART_GetDataSize());
	while (UART_GetDataSize()) {
		int r = UART_GetByte(0);
		UART_ConsumeBytes(1);
		int len = read_data_line(r, buffer, max_line_length);
		//printf("Len %i, buffer[3] = %i \n", len, buffer[3]);
		if (len == sizeof(m_get_cmd_resp) && buffer[3] == 0x04) {
			memcpy(m_get_cmd_resp.raw, buffer, len);
			print_hex_str(buffer, len);
			if (is_valid_xor(buffer, len)) {
				float curr_temp = (((buffer[17] << 8) | buffer[18]) / 374 - 32) / 1.8;
				is_changed = false;

				ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "Ok we got reply with mode %i, fan %i, turbo %i, mute %i",
					(int)m_get_cmd_resp.data.power, (int)m_get_cmd_resp.data.fan,
					(int)m_get_cmd_resp.data.turbo, (int)m_get_cmd_resp.data.mute);

				// Only 3 modes exposed: off, heat_cool, fan_only
				// All heating/cooling modes map to heat_cool
				if (m_get_cmd_resp.data.power == 0x00) {
					set_mode(CLIMATE_MODE_OFF);
					g_heat_cool_mode = false;
				} else if (g_heat_cool_mode) {
					// In heat_cool virtual mode — keep reporting heat_cool regardless
					// of what the hardware sub-mode is (cool/heat/fan_only are all
					// internal states managed by TCL_ApplyHeatCoolLogic)
					set_mode(CLIMATE_MODE_HEAT_COOL);
				} else if (m_get_cmd_resp.data.mode == 0x02) {
					set_mode(CLIMATE_MODE_FAN_ONLY);
				} else {
					// Hardware in cool/heat/dry/auto — this means heat_cool was active
					// before reboot. Re-enter heat_cool mode so deadband logic runs.
					g_heat_cool_mode = true;
					g_heat_cool_last_action = m_get_cmd_resp.data.mode;  // preserve current action for hysteresis
					set_mode(CLIMATE_MODE_HEAT_COOL);
				}

				if (m_get_cmd_resp.data.turbo) set_custom_fan_mode((FAN_TURBO));
				else if (m_get_cmd_resp.data.mute) set_custom_fan_mode((FAN_MUTE));
				else if (m_get_cmd_resp.data.fan == 0x00) set_custom_fan_mode((FAN_AUTOMATIC));
				else if (m_get_cmd_resp.data.fan == 0x01) set_custom_fan_mode((FAN_1));
				else if (m_get_cmd_resp.data.fan == 0x04) set_custom_fan_mode((FAN_2));
				else if (m_get_cmd_resp.data.fan == 0x02) set_custom_fan_mode((FAN_3));
				else if (m_get_cmd_resp.data.fan == 0x05) set_custom_fan_mode((FAN_4));
				else if (m_get_cmd_resp.data.fan == 0x03) set_custom_fan_mode((FAN_5));


				/* if (m_get_cmd_resp.data.hswing && m_get_cmd_resp.data.vswing) set_swing_mode(CLIMATE_SWING_BOTH);
				else if (!m_get_cmd_resp.data.hswing && !m_get_cmd_resp.data.vswing) set_swing_mode(CLIMATE_SWING_OFF);
				else if (m_get_cmd_resp.data.vswing) set_swing_mode(CLIMATE_SWING_VERTICAL);
				else if (m_get_cmd_resp.data.hswing) set_swing_mode(CLIMATE_SWING_HORIZONTAL);*/

				if (m_get_cmd_resp.data.vswing_mv == 0x01) set_swingV(VS_MoveFull);
				else if (m_get_cmd_resp.data.vswing_mv == 0x02) set_swingV(VS_MoveUpper);
				else if (m_get_cmd_resp.data.vswing_mv == 0x03) set_swingV(VS_MoveLower);
				else if (m_get_cmd_resp.data.vswing_fix == 0x01) set_swingV(VS_FixTop);
				else if (m_get_cmd_resp.data.vswing_fix == 0x02) set_swingV(VS_FixUpper);
				else if (m_get_cmd_resp.data.vswing_fix == 0x03) set_swingV(VS_FixMid);
				else if (m_get_cmd_resp.data.vswing_fix == 0x04) set_swingV(VS_FixLower);
				else if (m_get_cmd_resp.data.vswing_fix == 0x05) set_swingV(VS_FixBottom);
				else {
					//set_swingV("Last position");
				}

				if (m_get_cmd_resp.data.hswing_mv == 0x01) set_swingH(HS_MOVE_FULL);
				else if (m_get_cmd_resp.data.hswing_mv == 0x02) set_swingH(HS_MOVE_LEFT);
				else if (m_get_cmd_resp.data.hswing_mv == 0x03) set_swingH(HS_MOVE_MID);
				else if (m_get_cmd_resp.data.hswing_mv == 0x04) set_swingH(HS_MOVE_RIGHT);
				else if (m_get_cmd_resp.data.hswing_fix == 0x01) set_swingH(HS_FIX_LEFT);
				else if (m_get_cmd_resp.data.hswing_fix == 0x02) set_swingH(HS_FIX_MID_LEFT);
				else if (m_get_cmd_resp.data.hswing_fix == 0x03) set_swingH(HS_FIX_MID);
				else if (m_get_cmd_resp.data.hswing_fix == 0x04) set_swingH(HS_FIX_MID_RIGHT);
				else if (m_get_cmd_resp.data.hswing_fix == 0x05) set_swingH(HS_FIX_RIGHT);
				else {
					//set_swingH("Last position");
				}

				ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "fan %02X", m_get_cmd_resp.data.fan);
				ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "mode %02X", m_get_cmd_resp.data.mode);
				set_target_temperature((float)(m_get_cmd_resp.data.temp + 16));
				set_current_temperature(curr_temp);

				// Re-evaluate heat_cool switching on every temp update
				if (g_heat_cool_mode) {
					TCL_ApplyHeatCoolLogic();
				}

				if (is_changed)
				{
					//publish_state();
				}
			}
			//publish_state(buffer);
		}
	}
}

// Heat/cool auto-switching logic — ported from ESPHome tcl_climate component.
// Compares current_temperature against the two setpoints and sends the
// appropriate single-setpoint heat or cool command to the hardware.
// Uses 1°C hysteresis to prevent rapid cycling at boundary temps.

void TCL_ApplyHeatCoolLogic(void) {
	if (!g_heat_cool_mode)
		return;

	float temp = current_temperature;

	uint8_t want_mode;
	float want_temp;

	// Hysteresis: once cooling, keep cooling until temp drops 1°C below high setpoint.
	// Once heating, keep heating until temp rises 1°C above low setpoint.
	// Once idle, stay idle until temp crosses a setpoint boundary.
	if (g_heat_cool_last_action == 0x01) {
		// Was cooling — stop only when temp <= high - 1
		if (temp <= g_heat_cool_high - 1.0f) {
			// Satisfied — go idle
			want_mode = 0x02;
		} else {
			// Keep cooling
			want_mode = 0x01;
			want_temp = g_heat_cool_high;
		}
	} else if (g_heat_cool_last_action == 0x04) {
		// Was heating — stop only when temp >= low + 1
		if (temp >= g_heat_cool_low + 1.0f) {
			// Satisfied — go idle
			want_mode = 0x02;
		} else {
			// Keep heating
			want_mode = 0x04;
			want_temp = g_heat_cool_low;
		}
	} else {
		// Was idle (or first run) — only start when temp crosses a boundary
		if (temp > g_heat_cool_high) {
			want_mode = 0x01;  // start cooling
			want_temp = g_heat_cool_high;
		} else if (temp < g_heat_cool_low) {
			want_mode = 0x04;  // start heating
			want_temp = g_heat_cool_low;
		} else {
			want_mode = 0x02;  // stay idle
		}
	}

	if (want_mode == 0x02) {
		// Idle (fan_only) — stop compressor
		g_heat_cool_last_action = 0x02;

		if (m_get_cmd_resp.data.power && m_get_cmd_resp.data.mode == 0x02) {
			// Already in fan_only — nothing to do
			return;
		}

		ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "heat_cool: temp=%.1f in deadband [%.0f, %.0f] -> fan_only",
			temp, g_heat_cool_low, g_heat_cool_high);

		get_cmd_resp_t working = { 0 };
		memcpy(working.raw, m_get_cmd_resp.raw, sizeof(working.raw));
		working.data.power = 1;
		working.data.mode = 0x02;  // fan_only — compressor off
		// Keep the high setpoint as display temp to avoid flicker
		working.data.temp = (uint8_t)g_heat_cool_high - 16;
		build_set_cmd(&working);
		ready_to_send_set_cmd_flag = true;
		return;
	}

	// Active heating or cooling
	g_heat_cool_last_action = want_mode;

	uint8_t want_temp_raw = (uint8_t)want_temp - 16;
	if (m_get_cmd_resp.data.power == 1 && m_get_cmd_resp.data.mode == want_mode && m_get_cmd_resp.data.temp == want_temp_raw)
		return;

	ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "heat_cool: temp=%.1f low=%.0f high=%.0f -> %s @ %.0f",
		temp, g_heat_cool_low, g_heat_cool_high,
		want_mode == 0x04 ? "heat" : "cool", want_temp);

	get_cmd_resp_t working = { 0 };
	memcpy(working.raw, m_get_cmd_resp.raw, sizeof(working.raw));
	working.data.power = 1;
	working.data.mode = want_mode;
	working.data.temp = want_temp_raw;

	build_set_cmd(&working);
	ready_to_send_set_cmd_flag = true;
}

static commandResult_t CMD_ACMode(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int mode;

	Tokenizer_TokenizeString(args, 0);

	mode = parseClimate(Tokenizer_GetArg(0));
	OBK_SetClimate(mode);
	return CMD_RES_OK;
}
static commandResult_t CMD_FANMode(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int mode;

	Tokenizer_TokenizeString(args, 0);

	mode = parseFanMode(Tokenizer_GetArg(0));
	OBK_SetFanMode(mode);
	return CMD_RES_OK;
}

void HTTP_CreateSelect(http_request_t *request, const char **options, int numOptions, const char *active, const char *command) {
	// on select, send option string to /cm?cmnd=Command [Option]
	char tmpA[64];
	if (http_getArg(request->url, command, tmpA, sizeof(tmpA))) {
		CMD_ExecuteCommandArgs(command, tmpA, 0);
		// hack for display?
		active = tmpA;
	}
	hprintf255(request,
		"<form method='get'>"
		"<select name='%s' onchange='this.form.submit()'>", command);

	for (int i = 0; i < numOptions; i++) {
		const char *selected = (strcmp(options[i], active) == 0) ? " selected" : "";
		hprintf255(request, "<option value=\"%s\"%s>%s</option>", options[i], selected, options[i]);
	}

	hprintf255(request, "</select></form>");
}
void HTTP_CreateDIV(http_request_t *request, const char *label) {

	hprintf255(request, "<div>%s</div>", label);
}
void HTTP_CreateRadio(http_request_t *request, const char **options, int numOptions, const char *active, const char *command) {
	char tmpA[64];
	if (http_getArg(request->url, command, tmpA, sizeof(tmpA))) {
		CMD_ExecuteCommandArgs(command, tmpA, 0);
		// hack for display?
		active = tmpA;
	}
	hprintf255(request, "<form method='get'>");
	hprintf255(request, "%s ", command);
	for (int i = 0; i < numOptions; i++) {
		const char *checked = (strcmp(options[i], active) == 0) ? " checked" : "";
		hprintf255(request,
			"<label><input type='radio' name='%s' value='%s'%s onchange='this.form.submit()'>%s</label> ",
			command, options[i], checked, options[i]);
	}

	hprintf255(request, "</form>");
}

void TCL_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
	if (bPreState) {
		hprintf255(request, "<div style=\"display: grid; grid-auto-flow: column;\">");
		HTTP_CreateDIV(request, "ACMode");
		HTTP_CreateDIV(request, "SwingV");
		HTTP_CreateDIV(request, "SwingH");
		hprintf255(request, "</div>");
		hprintf255(request, "<div style=\"display: grid; grid-auto-flow: column;\">");
		HTTP_CreateSelect(request, fanOptions, sizeof(fanOptions) / sizeof(fanOptions[0]), climateModeToStr(g_mode), "ACMode");
		HTTP_CreateSelect(request, vertical_swing_options, sizeof(vertical_swing_options) / sizeof(vertical_swing_options[0]), getSwingVLabel(g_swingV), "SwingV");
		HTTP_CreateSelect(request, horizontal_swing_options, sizeof(horizontal_swing_options) / sizeof(horizontal_swing_options[0]), getSwingHLabel(g_swingH),"SwingH");
		hprintf255(request, "</div>");
	}
	else {
		HTTP_CreateRadio(request, fanOptions, sizeof(fanOptions) / sizeof(fanOptions[0]), climateModeToStr(g_mode), "ACMode");
		HTTP_CreateRadio(request, vertical_swing_options, sizeof(vertical_swing_options) / sizeof(vertical_swing_options[0]), getSwingVLabel(g_swingV), "SwingV");
		HTTP_CreateRadio(request, horizontal_swing_options, sizeof(horizontal_swing_options) / sizeof(horizontal_swing_options[0]), getSwingHLabel(g_swingH), "SwingH");
		hprintf255(request, "<h3>SwingH: %s</h3>", getSwingHLabel(g_swingH));
		hprintf255(request, "<h3>SwingV: %s</h3>", getSwingVLabel(g_swingV));
		hprintf255(request, "<h3>Mode: %s</h3>", climateModeToStr(g_mode));
		hprintf255(request, "<h3>Current temperature: %f</h3>", current_temperature);
		hprintf255(request, "<h3>Target temperature: %f</h3>", target_temperature);
	}

}
static commandResult_t CMD_SwingH(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int mode;

	Tokenizer_TokenizeString(args, 0);

	mode = parse_horizontal_swing(Tokenizer_GetArg(0));
	control_horizontal_swing(mode);
	return CMD_RES_OK;
}
static commandResult_t CMD_TargetTemperature(const void* context, const char* cmd, const char* args, int cmdFlags) {
	float target;

	Tokenizer_TokenizeString(args, 0);

	target = Tokenizer_GetArgFloat(0);
	OBK_SetTargetTemperature(target);
	return CMD_RES_OK;
}
static commandResult_t CMD_TargetTempLow(const void* context, const char* cmd, const char* args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	float val = Tokenizer_GetArgFloat(0);
	// HA sends °C (per discovery temp_unit), but guard against °F values
	if (val > 45.0f) val = (val - 32.0f) * 5.0f / 9.0f;
	g_heat_cool_low = val;
	ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "TargetTempLow set to %.1f C", g_heat_cool_low);
	HAL_FlashVars_SaveChannel(TCL_FLASH_CH_HEAT_COOL_LOW, (int)(g_heat_cool_low * 2.0f));
	if (g_heat_cool_mode) {
		TCL_ApplyHeatCoolLogic();
	}
	return CMD_RES_OK;
}
static commandResult_t CMD_TargetTempHigh(const void* context, const char* cmd, const char* args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	float val = Tokenizer_GetArgFloat(0);
	if (val > 45.0f) val = (val - 32.0f) * 5.0f / 9.0f;
	g_heat_cool_high = val;
	ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "TargetTempHigh set to %.1f C", g_heat_cool_high);
	HAL_FlashVars_SaveChannel(TCL_FLASH_CH_HEAT_COOL_HIGH, (int)(g_heat_cool_high * 2.0f));
	if (g_heat_cool_mode) {
		TCL_ApplyHeatCoolLogic();
	}
	return CMD_RES_OK;
}
static commandResult_t CMD_SwingV(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int mode;

	Tokenizer_TokenizeString(args, 0);

	mode = parse_vertical_swing(Tokenizer_GetArg(0));
	control_vertical_swing(mode);
	return CMD_RES_OK;
}
static commandResult_t CMD_Display(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int display;

	Tokenizer_TokenizeString(args, 0);

	display = Tokenizer_GetArgInteger(0);
	OBK_SetDisplay(display);
	return CMD_RES_OK;
}
static commandResult_t CMD_Gen(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int gen;

	Tokenizer_TokenizeString(args, 0);

	gen = Tokenizer_GetArgInteger(0);
	OBK_SetGen(gen);
	return CMD_RES_OK;
}
static commandResult_t CMD_Buzzer(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int buzzer;

	Tokenizer_TokenizeString(args, 0);

	buzzer = Tokenizer_GetArgInteger(0);
	OBK_SetBuzzer(buzzer);
	return CMD_RES_OK;
}
void TCL_Init(void) {

	UART_InitUART(TCL_baudRate, 2, false);
	UART_InitReceiveRingBuffer(TCL_UART_RECEIVE_BUFFER_SIZE);

	// Restore heat_cool state from flash (survives hardware watchdog reboots)
	{
		int stored_low = HAL_FlashVars_GetChannelValue(TCL_FLASH_CH_HEAT_COOL_LOW);
		int stored_high = HAL_FlashVars_GetChannelValue(TCL_FLASH_CH_HEAT_COOL_HIGH);
		int stored_mode = HAL_FlashVars_GetChannelValue(TCL_FLASH_CH_HEAT_COOL_MODE);
		// Values stored as temp*2 (to preserve 0.5 step). 0 = never written.
		if (stored_low >= 30 && stored_low <= 60) {  // 15.0-30.0 °C range
			g_heat_cool_low = (float)stored_low / 2.0f;
		}
		if (stored_high >= 30 && stored_high <= 60) {
			g_heat_cool_high = (float)stored_high / 2.0f;
		}
		if (stored_mode == 1) {
			g_heat_cool_mode = true;
		}
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER,
			"TCL: restored heat_cool from flash: low=%.1f high=%.1f mode=%d",
			g_heat_cool_low, g_heat_cool_high, (int)g_heat_cool_mode);
	}

	//cmddetail:{"name":"ACMode","args":"[Mode]",
	//cmddetail:"descr":"Sets the climate mode (off, cool, dry, fan_only, heat, heatcool, auto)",
	//cmddetail:"fn":"CMD_ACMode","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("ACMode", CMD_ACMode, NULL);
	//cmddetail:{"name":"FANMode","args":"CMD_FANMode",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"CMD_FANMode","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("FANMode", CMD_FANMode, NULL);
	//cmddetail:{"name":"SwingH","args":"CMD_SwingH",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"CMD_SwingH","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("SwingH", CMD_SwingH, NULL);
	//cmddetail:{"name":"SwingV","args":"CMD_SwingV",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"CMD_SwingV","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("SwingV", CMD_SwingV, NULL);
	//cmddetail:{"name":"TargetTemperature","args":"CMD_TargetTemperature",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"CMD_TargetTemperature","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("TargetTemperature", CMD_TargetTemperature, NULL);
	//cmddetail:{"name":"Buzzer","args":"CMD_Buzzer",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"CMD_Buzzer","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("Buzzer", CMD_Buzzer, NULL);

	//cmddetail:{"name":"Gen","args":"Gen",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"CMD_Gen","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("Gen", CMD_Gen, NULL);
	
	//cmddetail:{"name":"Display","args":"CMD_Display",
	//cmddetail:"descr":"",
	//cmddetail:"fn":"CMD_Display","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("Display", CMD_Display, NULL);

	//cmddetail:{"name":"TargetTempLow","args":"[Temperature]",
	//cmddetail:"descr":"Sets the heat-to setpoint for heat_cool mode (°C)",
	//cmddetail:"fn":"CMD_TargetTempLow","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("TargetTempLow", CMD_TargetTempLow, NULL);
	//cmddetail:{"name":"TargetTempHigh","args":"[Temperature]",
	//cmddetail:"descr":"Sets the cool-to setpoint for heat_cool mode (°C)",
	//cmddetail:"fn":"CMD_TargetTempHigh","file":"driver/drv_tclAC.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("TargetTempHigh", CMD_TargetTempHigh, NULL);
}

// Compute the current HVAC action string for HA
static const char* TCL_ComputeAction(void) {
	if (m_get_cmd_resp.data.power == 0)
		return "off";
	if (g_heat_cool_mode) {
		// In heat_cool mode, report based on our commanded sub-mode
		switch (g_heat_cool_last_action) {
			case 0x01: return "cooling";
			case 0x04: return "heating";
			default: return "idle";  // fan_only = deadband satisfied
		}
	}
	switch (m_get_cmd_resp.data.mode) {
		case 0x01: return "cooling";
		case 0x04: return "heating";
		case 0x03: return "drying";
		case 0x02: return "fan";
		case 0x05: {
			// Auto mode — infer from current temp vs target
			float target = (float)(m_get_cmd_resp.data.temp + 16);
			if (current_temperature > target + 1.0f)
				return "cooling";
			else if (current_temperature < target - 1.0f)
				return "heating";
			return "idle";
		}
		default: return "idle";
	}
}

// backlog startDriver TCL; Gen 3
void TCL_UART_RunEverySecond(void) {
	uint8_t req_cmd[] = { 0xBB, 0x00, 0x01, 0x04, 0x02, 0x01, 0x00, 0xBD };

	// Staggered publish: one slot per second, rotating through 10 values.
	// Immediate publish on change for responsiveness.
	// This prevents lwIP ERR_MEM from accumulating (which caused reconnects every ~5 min).
	int slot = tcl_publish_slot;
	tcl_publish_slot = (tcl_publish_slot + 1) % TCL_PUBLISH_SLOT_COUNT;

	int cur_temp = (int)current_temperature;
	int cur_low = (int)g_heat_cool_low;
	int cur_high = (int)g_heat_cool_high;

	if (slot == 0 || cur_temp != tcl_prev_current_temp) {
		MQTT_PublishMain_StringInt("CurrentTemperature", cur_temp, 0);
		tcl_prev_current_temp = cur_temp;
	}
	if (slot == 1 || cur_low != tcl_prev_target_low) {
		MQTT_PublishMain_StringInt("TargetTempLow", cur_low, 0);
		tcl_prev_target_low = cur_low;
	}
	if (slot == 2 || cur_high != tcl_prev_target_high) {
		MQTT_PublishMain_StringInt("TargetTempHigh", cur_high, 0);
		tcl_prev_target_high = cur_high;
	}
	if (slot == 3 || g_mode != tcl_prev_mode) {
		MQTT_PublishMain_StringString("ACMode", climateModeToStr(g_mode), 0);
		tcl_prev_mode = g_mode;
	}
	if (slot == 4 || g_fanMode != tcl_prev_fan) {
		MQTT_PublishMain_StringString("FANMode", fanModeToStr(g_fanMode), 0);
		tcl_prev_fan = g_fanMode;
	}
	if (slot == 5 || g_buzzer != tcl_prev_buzzer) {
		MQTT_PublishMain_StringInt("Buzzer", g_buzzer, 0);
		tcl_prev_buzzer = g_buzzer;
	}
	if (slot == 6 || g_disp != tcl_prev_disp) {
		MQTT_PublishMain_StringInt("Display", g_disp, 0);
		tcl_prev_disp = g_disp;
	}
	if (slot == 7 || g_swingH != tcl_prev_swingH) {
		MQTT_PublishMain_StringString("SwingH", getSwingHLabel(g_swingH), 0);
		tcl_prev_swingH = g_swingH;
	}
	if (slot == 8 || g_swingV != tcl_prev_swingV) {
		MQTT_PublishMain_StringString("SwingV", getSwingVLabel(g_swingV), 0);
		tcl_prev_swingV = g_swingV;
	}

	// HVAC action (what the unit is actually doing)
	const char* cur_action = TCL_ComputeAction();
	if (slot == 9 || cur_action != tcl_prev_action) {
		MQTT_PublishMain_StringString("HVACAction", cur_action, 0);
		tcl_prev_action = cur_action;
	}

	if (ready_to_send_set_cmd_flag) {
		ADDLOG_WARN(LOG_FEATURE_ENERGYMETER, "Sending data");
		ready_to_send_set_cmd_flag = false;
		write_array(m_set_cmd.raw, sizeof(m_set_cmd.raw));
	}
	else {
		write_array(req_cmd, sizeof(req_cmd));
	}
	TCL_UART_TryToGetNextPacket();
}
#include "../httpserver/hass.h"
// backlog startDriver TCL; scheduleHADiscovery
void TCL_DoDiscovery(const char *topic) {
	HassDeviceInfo* dev_info = NULL;


	dev_info = hass_createHVAC(15,30,0.5f, fanOptions, sizeof(fanOptions)/sizeof(fanOptions[0]),
		vertical_swing_options,sizeof(vertical_swing_options) / sizeof(vertical_swing_options[0]),
		horizontal_swing_options, sizeof(horizontal_swing_options) / sizeof(horizontal_swing_options[0])
		);
	MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
	hass_free_device_info(dev_info);

	//dev_info = hass_createFanWithModes("Fan Speed", "~/FANMode/get", "FANMode", fanOptions, 4);
	//MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
	//hass_free_device_info(dev_info);

	// Buzzer and Display switch entities — build minimal discovery JSON
	// directly instead of using hass_createToggle, which produces payloads
	// that HA silently ignores for some device configurations.
	{
		const char *toggles[][3] = {
			{ "Buzzer",  "tcl_buzzer",  "Buzzer"  },
			{ "Display", "tcl_display", "Display" },
		};
		int i;
		for (i = 0; i < 2; i++) {
			const char *label   = toggles[i][0];
			const char *suffix  = toggles[i][1];
			const char *command = toggles[i][2];

			char uniq[64], cmd_t[64], stat_t[64], avty_t[64], chan[128];
			snprintf(uniq,   sizeof(uniq),   "%s_%s", CFG_GetMQTTClientId(), suffix);
			snprintf(cmd_t,  sizeof(cmd_t),  "cmnd/%s/%s", CFG_GetMQTTClientId(), command);
			snprintf(stat_t, sizeof(stat_t), "%s/%s/get", CFG_GetMQTTClientId(), command);
			snprintf(avty_t, sizeof(avty_t), "%s/connected", CFG_GetMQTTClientId());
			snprintf(chan,   sizeof(chan),    "switch/%s/config", uniq);

			cJSON *root = cJSON_CreateObject();
			cJSON *dev  = cJSON_CreateObject();
			cJSON *ids  = cJSON_CreateArray();
			cJSON_AddItemToArray(ids, cJSON_CreateString(CFG_GetDeviceName()));
			cJSON_AddItemToObject(dev, "ids", ids);
			cJSON_AddStringToObject(dev, "name", CFG_GetShortDeviceName());

			cJSON_AddItemToObject(root, "dev", dev);
			cJSON_AddStringToObject(root, "name",    label);
			cJSON_AddStringToObject(root, "uniq_id", uniq);
			cJSON_AddStringToObject(root, "cmd_t",   cmd_t);
			cJSON_AddStringToObject(root, "stat_t",  stat_t);
			cJSON_AddStringToObject(root, "pl_on",   "1");
			cJSON_AddStringToObject(root, "pl_off",  "0");
			cJSON_AddStringToObject(root, "avty_t",  avty_t);

			char *json = cJSON_PrintUnformatted(root);
			MQTT_QueuePublish(topic, chan, json, OBK_PUBLISH_FLAG_RETAIN);
			os_free(json);
			cJSON_Delete(root);
		}
	}


		//char command_topic[64];

		//// Vertical Swing Entity
		//sprintf(command_topic, "cmnd/%s/SwingV", CFG_GetMQTTClientId());
		//dev_info = hass_createSelectEntity(
		//	"~/SwingV/get",               // state_topic
		//	command_topic,                          // command_topic
		//	9,                                      // numoptions (VerticalSwingMode has 9 values)
		//	vertical_swing_options,                 // fanOptions array
		//	"Vertical Swing Mode"                   // title
		//);
		//MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		//hass_free_device_info(dev_info);

		//// Horizontal Swing Entity
		//sprintf(command_topic, "cmnd/%s/SwingH", CFG_GetMQTTClientId());
		//dev_info = hass_createSelectEntity(
		//	"~/SwingH/get",            // state_topic
		//	command_topic,                          // command_topic
		//	10,                                     // numoptions (HorizontalSwing has 10 values)
		//	horizontal_swing_options,               // fanOptions array
		//	"Horizontal Swing Mode"                 // title
		//);
	//	MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		//hass_free_device_info(dev_info);

}

#endif
