import time
import math
import ujson
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from rotary_irq_rp2 import RotaryIRQ
import image_rp #raspberry pi pico logo
import image_ad #analog devices logo
import framebuf

I2C_ADDR = 0x34  # ADAU1701 I2C address
DATA_REG = 0x810
ADDR_REG = 0x815
GAIN_ADDR = 0x001 # Single coefficient. Example: -80db [0x03 0x47] -40db [0x01 0x47 0xAE] 0db [0x80 0x00 0x00]
VOLUME_ADDR = 0x002 # Single coefficient. Example: -80db [0x03 0x47] -40db [0x01 0x47 0xAE] 0db [0x80 0x00 0x00]
PHASE_ADDR = 0x003 # Two coefficients. Either 0 degrees: [0x80 0x00 0x00] or 180 degrees: [0xFF 0x80 0x00 0x00]
SUBSONIC_ADDR_1 = 0x004 # First stage subsonic coefficients. Five coefficients
SUBSONIC_ADDR_2 = 0x009 # Second stage subsonic coefficients. Five coefficients
PARAM_EQ_ADDR_1 = 0x00E # Five coefficients
PARAM_EQ_ADDR_2 = 0x013 # Five coefficients
PARAM_EQ_ADDR_3 = 0x018 # Five coefficients
PARAM_EQ_ADDR_4 = 0x01D # Five coefficients
PARAM_EQ_ADDR_5 = 0x022 # Five coefficients
PARAM_EQ_ADDR = [0x00E, 0x013, 0x018, 0x01D, 0x022]
LOWPASS_ADDR_1 = 0x027 # First Linkwitz-Riley 2nd order address. Five coefficients
LOWPASS_ADDR_2 = 0x02C # Second Linkwitz-Riley 2nd order address. Five coefficients. Coefficients are the same as in lowpass_addr_1

# === Constants ===
FS = 48000  # Sample rate in Hz
ADAU1701_ADDR = 0x34  # ADAU1701 I2C address (for future use)
MAX_VISIBLE = 5  # Maximum visible menu items (adjust based on display height)
SAVE_DELAY_MS = 2000  # Delay before saving presets to flash (2 seconds)

i2c_adau = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

# === I2C OLED Setup ===
#i2c = I2C(0, scl=Pin(5), sda=Pin(4))
i2c_oled = I2C(1, scl=Pin(3), sda=Pin(2))
oled = SSD1306_I2C(128, 64, i2c_oled)

# === Rotary Encoder & Button Setup ===
SW = Pin(6, Pin.IN, Pin.PULL_UP)
r = RotaryIRQ(
    pin_num_clk=8,
    pin_num_dt=7,
    min_val=0,
    max_val=1000,
    reverse=True,
    range_mode=RotaryIRQ.RANGE_UNBOUNDED,
    half_step=False
)

# === Screen Saver ===
dim_after_ms = 10000
off_after_ms = 20000
oled_on = True
last_activity = time.ticks_ms()

def wake_oled():
    global oled_on, last_activity
    last_activity = time.ticks_ms()
    oled.contrast(255)  # Restore full brightness
    oled_on = True
    display_menu()

def handle_oled_timeout():
    global oled_on
    now = time.ticks_ms()
    idle_time = time.ticks_diff(now, last_activity)
    if idle_time > off_after_ms and oled_on:
        oled.fill(0)
        oled.show()
        oled.contrast(0)
        oled_on = False
    elif idle_time > dim_after_ms and oled_on:
        oled.contrast(10)

def parametric_eq(frequency, q, boost):
    gain = 0
    gainlinear = 10 ** (gain / 20)
    if boost == 0:
        b0 = gainlinear
        b1 = 0
        b2 = 0
        a1 = 0
        a2 = 0
    else:
        ax = 10 ** (boost / 40)
        omega = 2 * math.pi * frequency / FS
        sn = math.sin(omega)
        cs = math.cos(omega)
        alpha = sn / (2 * q)
        a0 = 1 + (alpha / ax)
        a1 = -(2 * cs) / a0
        a2 = (1 - (alpha / ax)) / a0
        gainlinear = 10 ** (gain/20) / a0
        b0 = (1 + (alpha * ax)) * gainlinear
        b1 = -( 2 * cs) * gainlinear
        b2 = (1 - (alpha * ax)) * gainlinear
    return b0, b1, b2, a1, a2
    
def linkwitz_riley_lowpass(frequency, FS):
    gain = 0 #no gain needed
    omega = 2 * math.pi * frequency / FS
    sn = math.sin(omega)
    cs = math.cos(omega)
    alpha = sn/(2 * (1 / 2 ** 0.5))
    a0 = 1 + alpha
    a1 = -(2 * cs) / a0
    a2 = (1 - alpha) / a0
    b1 = (1 - cs) / a0 * 10 ** (gain / 20)
    b0 = b1 / 2
    b2 = b0
    return b0,b1,b2,a1,a2
    
def butterworth_subsonic_24(fc, FS, i):
    orderindex=4 #24dB per octave
    gain = 0 #no gain needed
    omega = 2 * math.pi * fc / FS
    sn = math.sin(omega)
    cs = math.cos(omega)
    orderangle = (math.pi / orderindex) * (i + 0.5)
    alpha = sn / (2 * (1 / (2 * math.sin(orderangle))))
    a0 = 1 + alpha
    a1 = -(2 * cs) / a0
    a2 = (1 - alpha) / a0
    b1 = -(1 + cs) / a0 * 10 ** (gain / 20)
    b0 = - b1 / 2
    b2 = b0
    return b0,b1,b2,a1,a2

def volume(attenuation_db):
    """
    Calculate attenuated value from 0 dB reference (2^23) and return in 5.23 fixed-point format
    as four separate hex bytes (MSB to LSB).
    """
    full_scale_value = 2 ** 23
    linear_factor = 10 ** (attenuation_db / 20)
    attenuated_value = full_scale_value * linear_factor

    # Convert to 5.23 fixed-point format
    fixed_point_value = int(attenuated_value) & 0xFFFFFFFF

    # Extract four bytes (MSB to LSB)
    byte1 = (fixed_point_value >> 24) & 0xFF
    byte2 = (fixed_point_value >> 16) & 0xFF
    byte3 = (fixed_point_value >> 8) & 0xFF
    byte4 = fixed_point_value & 0xFF

    return [hex(byte1), hex(byte2), hex(byte3), hex(byte4)]

def phase_calc(phase):
    if phase == 0:
        return [hex(0x00), hex(0x80), hex(0x00), hex(0x00)]
    else:
        return [hex(0xFF), hex(0x80), hex(0x00), hex(0x00)]
    
def write_safeload(addr_coeff, biquad, index):
    #format data register array
    byte_values = [int(h, 16) for h in biquad]
    biquad_array = bytearray(byte_values)
    data_reg_array = bytearray([(DATA_REG >> 8) & 0xFF, DATA_REG & 0xFF])
    data_reg_array[-1] = (data_reg_array[-1] + index) & 0xFF
    combined_data=bytes(data_reg_array)+bytes([0x00])+bytes(biquad_array)
    
    #format addr register array
    addr_reg_array = bytearray([(ADDR_REG >> 8) & 0xFF, ADDR_REG & 0xFF])
    addr_reg_array[-1] = (addr_reg_array[-1] + index) & 0xFF
    addr_coeff_array = bytearray([(addr_coeff >> 8) & 0xFF, addr_coeff & 0xFF])
    addr_coeff_array[-1] = (addr_coeff_array[-1] + index) & 0xFF
    combined_addr=bytes(addr_reg_array)+bytes(addr_coeff_array)
    #write to ADAU1701
    i2c_adau.writeto(I2C_ADDR, combined_data)
    i2c_adau.writeto(I2C_ADDR, combined_addr)

def trigger_safeload():
    i2c_adau.writeto(I2C_ADDR, bytes([0x08, 0x1c, 0x00, 0x3c]))

def float_to_fixed_point_bytes(value):
    # Fixed-point format: 5 integer bits, 23 fractional bits
    total_bits = 28
    fractional_bits = 23
    scale = 2 ** fractional_bits

    # Scale the float to fixed-point integer
    scaled_value = int(round(value * scale))

    # Apply two's complement if negative
    if scaled_value < 0:
        scaled_value = (1 << total_bits) + scaled_value

    # Mask to ensure it's within 28 bits
    scaled_value &= (1 << total_bits) - 1

    # Convert to 4 bytes (big-endian)
    byte_array = scaled_value.to_bytes(4, 'big')

    # Format each byte as hex
    hex_bytes = [f"0x{byte:02X}" for byte in byte_array]
    return hex_bytes

def coeffs_to_array(b0,b1,b2,a1,a2):
    a1_hex = float_to_fixed_point_bytes(-a1)
    a2_hex = float_to_fixed_point_bytes(-a2) 
    b0_hex = float_to_fixed_point_bytes(b0)
    b1_hex = float_to_fixed_point_bytes(b1)
    b2_hex = float_to_fixed_point_bytes(b2)
    return b0_hex, b1_hex, b2_hex, a1_hex, a2_hex
   
# === Preset Management ===
presets = [
    {
        "name": "PR1", 
        "volume_db": -12,
        "gain_db": -3,
        "phase": 0, 
        "lp_freq": 80, 
        "hp_freq": 80,
        "eq_settings": [{"freq": 100, "q": 1.0, "boost": 0} for _ in range(5)],
        "lp_biquads": [linkwitz_riley_lowpass(80, FS), linkwitz_riley_lowpass(80, FS)],
        "hp_biquads": [butterworth_subsonic_24(80, FS, 1), butterworth_subsonic_24(80, FS, 0)],
        "eq_biquads": [parametric_eq(100, 1.0, 0) for _ in range(5)]
    },
    {
        "name": "PR2", 
        "volume_db": -12, 
        "gain_db": -3,
        "phase": 0, 
        "lp_freq": 80, 
        "hp_freq": 80,
        "eq_settings": [{"freq": 100, "q": 1.0, "boost": 0} for _ in range(5)],
        "lp_biquads": [linkwitz_riley_lowpass(80, FS), linkwitz_riley_lowpass(80, FS)],
        "hp_biquads": [butterworth_subsonic_24(80, FS, 1), butterworth_subsonic_24(80, FS, 0)],
        "eq_biquads": [parametric_eq(100, 1.0, 0) for _ in range(5)]
    },
    {
        "name": "PR3", 
        "volume_db": -12,
        "gain_db": -3, 
        "phase": 0, 
        "lp_freq": 80, 
        "hp_freq": 80,
        "eq_settings": [{"freq": 100, "q": 1.0, "boost": 0} for _ in range(5)],
        "lp_biquads": [linkwitz_riley_lowpass(80, FS), linkwitz_riley_lowpass(80, FS)],
        "hp_biquads": [butterworth_subsonic_24(80, FS, 1), butterworth_subsonic_24(80, FS, 0)],
        "eq_biquads": [parametric_eq(100, 1.0, 0) for _ in range(5)]
    },
    {
        "name": "PR4", 
        "volume_db": -12,
        "gain_db": -3, 
        "phase": 0, 
        "lp_freq": 80, 
        "hp_freq": 80,
        "eq_settings": [{"freq": 100, "q": 1.0, "boost": 0} for _ in range(5)],
        "lp_biquads": [linkwitz_riley_lowpass(80, FS), linkwitz_riley_lowpass(80, FS)],
        "hp_biquads": [butterworth_subsonic_24(80, FS, 1), butterworth_subsonic_24(80, FS, 0)],
        "eq_biquads": [parametric_eq(100, 1.0, 0) for _ in range(5)]
    },
    {
        "name": "PR5", 
        "volume_db": -12,
        "gain_db": -3, 
        "phase": 0, 
        "lp_freq": 80, 
        "hp_freq": 80,
        "eq_settings": [{"freq": 100, "q": 1.0, "boost": 0} for _ in range(5)],
        "lp_biquads": [linkwitz_riley_lowpass(80, FS), linkwitz_riley_lowpass(80, FS)],
        "hp_biquads": [butterworth_subsonic_24(80, FS, 1), butterworth_subsonic_24(80, FS, 0)],
        "eq_biquads": [parametric_eq(100, 1.0, 0) for _ in range(5)]
    }
]
current_preset = 0  # Index of the current preset (0–4 for PR1–PR5)
last_save_time = 0  # Timestamp of last parameter change
needs_save = False  # Flag to indicate pending save

def save_presets():
    global last_save_time, needs_save
    try:
        with open("/presets.json", "w") as f:
            ujson.dump({"presets": presets, "last_preset": current_preset}, f)
        last_save_time = time.ticks_ms()
        needs_save = False
        print("Presets saved to /presets.json")
    except OSError as e:
        print(f"Error saving presets: {e}")

def load_presets():
    global presets, current_preset, volume_db, gain_db, phase, lp_freq, hp_freq, eq_settings
    try:
        with open("/presets.json", "r") as f:
            data = ujson.load(f)
            loaded_presets = data["presets"]
            # Ensure backward compatibility: recompute biquads if missing
            for preset in loaded_presets:
                if "lp_biquads" not in preset:
                    preset["lp_biquads"] = [linkwitz_riley_lowpass(preset["lp_freq"], FS), linkwitz_riley_lowpass(preset["lp_freq"], FS)]
                if "hp_biquads" not in preset:
                    preset["hp_biquads"] = [butterworth_subsonic_24(preset["hp_freq"], FS, 1), butterworth_subsonic_24(preset["hp_freq"], FS, 0)]
                if "eq_biquads" not in preset:
                    preset["eq_biquads"] = [parametric_eq(eq["freq"], eq["q"], eq["boost"]) for eq in preset["eq_settings"]]
            presets = loaded_presets
            current_preset = data.get("last_preset", 0)
            apply_preset(current_preset)
    except OSError:
        print("No preset file found, using defaults")
        save_presets()

def apply_preset(preset_idx):
    global volume_db, phase, gain_db, lp_freq, hp_freq, eq_settings, current_preset
    preset = presets[preset_idx]
    volume_db = preset["volume_db"]
    phase = preset["phase"]
    gain_db = preset["gain_db"]
    lp_freq = preset["lp_freq"]
    hp_freq = preset["hp_freq"]
    eq_settings = preset["eq_settings"]
    current_preset = preset_idx

    # Apply 1st cutoff parameters from presets
    b0= preset["lp_biquads"][0][0]
    b1= preset["lp_biquads"][0][1]
    b2= preset["lp_biquads"][0][2]
    a1= preset["lp_biquads"][0][3]
    a2= preset["lp_biquads"][0][4]
    biquad_coeffs = coeffs_to_array(b0, b1, b2, a1, a2)
    for i in range(5):
        write_safeload(LOWPASS_ADDR_1, biquad_coeffs[i], i)
    trigger_safeload()

    # Apply 2nd cutoff parameters from presets
    b0= preset["lp_biquads"][1][0]
    b1= preset["lp_biquads"][1][1]
    b2= preset["lp_biquads"][1][2]
    a1= preset["lp_biquads"][1][3]
    a2= preset["lp_biquads"][1][4]
    biquad_coeffs = coeffs_to_array(b0, b1, b2, a1, a2)
    for i in range(5):
        write_safeload(LOWPASS_ADDR_2, biquad_coeffs[i], i)
    trigger_safeload()

    # Apply subsonic 1st stage parameters
    b0= preset["hp_biquads"][0][0]
    b1= preset["hp_biquads"][0][1]
    b2= preset["hp_biquads"][0][2]
    a1= preset["hp_biquads"][0][3]
    a2= preset["hp_biquads"][0][4]
    biquad_coeffs = coeffs_to_array(b0, b1, b2, a1, a2)
    for i in range(5):
        write_safeload(SUBSONIC_ADDR_1, biquad_coeffs[i], i)
    trigger_safeload()

    # Apply subsonic 2nd stage parameters
    b0= preset["hp_biquads"][1][0]
    b1= preset["hp_biquads"][1][1]
    b2= preset["hp_biquads"][1][2]
    a1= preset["hp_biquads"][1][3]
    a2= preset["hp_biquads"][1][4]
    biquad_coeffs = coeffs_to_array(b0, b1, b2, a1, a2)
    for i in range(5):
        write_safeload(SUBSONIC_ADDR_2, biquad_coeffs[i], i)
    trigger_safeload()

    # Apply EQ1 parameters
    for i in range (5):
        b0= preset["eq_biquads"][i][0]
        b1= preset["eq_biquads"][i][1]
        b2= preset["eq_biquads"][i][2]
        a1= preset["eq_biquads"][i][3]
        a2= preset["eq_biquads"][i][4]
        biquad_coeffs = coeffs_to_array(b0, b1, b2, a1, a2)
        for a in range(5):
            write_safeload(PARAM_EQ_ADDR[i], biquad_coeffs[a], a)
        trigger_safeload()

    # Apply volume
    #gain = 10 ** (volume_db / 20.0)
    write_safeload(VOLUME_ADDR, volume(volume_db),0)
    trigger_safeload()
    
    # Apply phase
    phase_conv = 180 if phase == 1 else 0
    write_safeload(PHASE_ADDR, phase_calc(phase_conv),0)
    trigger_safeload()
    print(f"Loaded preset {preset['name']}")
    save_presets()

    # Apply input gain
    #gain = 10 ** (volume_db / 20.0)
    write_safeload(GAIN_ADDR, volume(gain_db),0)
    trigger_safeload()

# === Menu Structure ===
menu_structure = [
    {"name": "Parameters", "children": [
        {"name": "Volume", "children": [{"name": "Value"}, {"name": "Back"}]},
        {"name": "Subsonic", "children": [{"name": "Freq"}, {"name": "Back"}]},
        {"name": "Param EQ", "children": [
            {"name": "EQ1", "children": [{"name": "Freq"}, {"name": "Q"}, {"name": "Boost"}, {"name": "Back"}]},
            {"name": "EQ2", "children": [{"name": "Freq"}, {"name": "Q"}, {"name": "Boost"}, {"name": "Back"}]},
            {"name": "EQ3", "children": [{"name": "Freq"}, {"name": "Q"}, {"name": "Boost"}, {"name": "Back"}]},
            {"name": "EQ4", "children": [{"name": "Freq"}, {"name": "Q"}, {"name": "Boost"}, {"name": "Back"}]},
            {"name": "EQ5", "children": [{"name": "Freq"}, {"name": "Q"}, {"name": "Boost"}, {"name": "Back"}]},
            {"name": "Back"}
        ]},
        {"name": "Cutoff", "children": [{"name": "Freq"}, {"name": "Back"}]},
        {"name": "Phase", "children": [{"name": "Value"}, {"name": "Back"}]},
        {"name": "Pre Gain", "children": [{"name": "Value"}, {"name": "Back"}]},
        {"name": "Back"}
    ]},
    {"name": "Presets", "children": [
        {"name": "Load preset", "children": [
            {"name": "PR1"}, {"name": "PR2"}, {"name": "PR3"}, {"name": "PR4"}, {"name": "PR5"}, {"name": "Back"}
        ]},
        {"name": "Rename preset"},
        {"name": "Back"}
    ]}
]

# === State Variables ===
menu_stack = [(menu_structure, 0, 0)]  # (menu, cursor, scroll_offset)
editing_parameter = False
volume_db = -12  # Initial volume in dB (-80 to 0 dB)
gain_db = -3
phase = 0  # 0 for 0°, 1 for 180°
lp_freq = 80  # Low-pass frequency (40–150 Hz)
hp_freq = 80  # High-pass frequency (10–40 Hz)
eq_settings = [{"freq": 100, "q": 1.0, "boost": 0} for _ in range(5)]
last_save_time = 0  # Timestamp of last parameter change
needs_save = False  # Flag to indicate pending save

# === Show boot logos and load Last Preset on Boot ===
fb_ad = framebuf.FrameBuffer(image_ad.ad, 128, 64, framebuf.MONO_VLSB)
fb_rp = framebuf.FrameBuffer(image_rp.rp, 128, 64, framebuf.MONO_VLSB)
oled.blit(fb_ad, 0, 0)
oled.show()
time.sleep_ms(2000)
oled.fill(0)
oled.blit(fb_rp, 0, 0)
oled.show()
load_presets()
time.sleep_ms(2000)
oled.fill(0)
oled.show()

# === Display Function ===
def display_menu():
    oled.fill(0)
    current_menu, cursor, scroll_offset = menu_stack[-1]
    if len(menu_stack) == 1:  # Top-level menu
        header_text = "Main menu"
        x_header = (128 - len(header_text) * 8) // 2  # Center header (8 pixels per char)
        oled.text(header_text, x_header, 0)
        preset_text = f"Preset: PR{current_preset + 1}"
        x_preset = (128 - len(preset_text) * 8) // 2  # Center preset (8 pixels per char)
        oled.text(preset_text, x_preset, 56)  # Bottom of 64-pixel display
    else:  # Sub-menus
        header_text = ""
        if len(menu_stack) >= 2:
            parent_item = menu_stack[-2][0][menu_stack[-2][1]]["name"]
            if parent_item in ["Volume", "Pre Gain", "Phase", "Cutoff", "Subsonic"]:
                header_text = f"{parent_item} {'[Edit]' if editing_parameter else ''}"
            elif parent_item.startswith("EQ"):
                header_text = f"{parent_item} {'[Edit]' if editing_parameter else ''}"
            elif parent_item == "Param EQ":
                header_text = "Param EQ"
            elif parent_item == "Load preset":
                header_text = f"Preset: {presets[current_preset]['name']}"
            elif parent_item == "Parameters":
                header_text = "Parameters"
            elif parent_item == "Presets":
                header_text = "Presets"
            x_preset = (128 - len(header_text) * 8) // 2  # Center preset (8 pixels per char)
            oled.text(header_text, x_preset, 0)

    end_idx = min(scroll_offset + MAX_VISIBLE, len(current_menu))
    for visible_idx, idx in enumerate(range(scroll_offset, end_idx)):
        item = current_menu[idx]
        marker = ">" if idx == cursor else " "
        x, y = 0, 16 + visible_idx * 10
        if item["name"] == "Value":
            parent_name = menu_stack[-2][0][menu_stack[-2][1]]["name"]
            if parent_name == "Volume":
                oled.text(f"{marker} Value: {volume_db}dB", x, y)
            elif parent_name == "Pre Gain":
                oled.text(f"{marker} Value: {gain_db}dB", x, y)
            elif parent_name == "Phase":
                phase_str = "180 deg" if phase == 1 else "0 deg"
                oled.text(f"{marker} Value: {phase_str}", x, y)
        elif item["name"] == "Freq":
            parent_name = menu_stack[-2][0][menu_stack[-2][1]]["name"]
            if parent_name == "Cutoff":
                oled.text(f"{marker} Freq: {lp_freq}Hz", x, y)
            elif parent_name == "Subsonic":
                oled.text(f"{marker} Freq: {hp_freq}Hz", x, y)
            elif parent_name.startswith("EQ"):
                eq_idx = menu_stack[-2][1]
                oled.text(f"{marker} Freq: {eq_settings[eq_idx]['freq']}Hz", x, y)
        elif item["name"] == "Q":
            eq_idx = menu_stack[-2][1]
            oled.text(f"{marker} Q: {eq_settings[eq_idx]['q']:.1f}", x, y)
        elif item["name"] == "Boost":
            eq_idx = menu_stack[-2][1]
            oled.text(f"{marker} Boost: {eq_settings[eq_idx]['boost']:.1f}dB", x, y)
        else:
            oled.text(f"{marker} {item['name']}", x, y)
    oled.show()

# === Button Handler ===
button_press_time = 0
def button_handler(pin):
    global button_press_time, editing_parameter, needs_save
    wake_oled()
    now = time.ticks_ms()
    if time.ticks_diff(now, button_press_time) < 800:
        return
    button_press_time = now
    current_menu, cursor, scroll_offset = menu_stack[-1]
    item = current_menu[cursor]
    if editing_parameter and item["name"] == "Back":
        editing_parameter = False
        save_presets()  # Save immediately when exiting editing mode
    elif editing_parameter:
        editing_parameter = False
    elif "children" in item:
        menu_stack.append((item["children"], 0, 0))
    elif item["name"] == "Back":
        if len(menu_stack) > 1:
            menu_stack.pop()
    elif item["name"] in ["Value", "Freq", "Q", "Boost"]:
        editing_parameter = True
    elif item["name"] in ["PR1", "PR2", "PR3", "PR4", "PR5"]:
        preset_idx = int(item["name"][2]) - 1
        apply_preset(preset_idx)
        menu_stack.pop()  # Return to Presets menu
        display_menu()
    elif item["name"] == "Rename preset":
        print("Rename preset not implemented")  # Placeholder
    display_menu()

# Register button interrupt
SW.irq(trigger=Pin.IRQ_RISING, handler=button_handler)

# === Rotary Handler ===
last_val = r.value()
def handle_rotary():
    global last_val, volume_db, phase, gain_db, lp_freq, hp_freq, eq_settings, needs_save, last_save_time
    val = r.value()
    delta = val - last_val
    if delta == 0:
        return
    last_val = val
    wake_oled()
    last_save_time = time.ticks_ms()  # Update last activity time for debounced save
    current_menu, cursor, scroll_offset = menu_stack[-1]
    item = current_menu[cursor]

    if editing_parameter:
        parent_name = menu_stack[-2][0][menu_stack[-2][1]]["name"]
        if item["name"] == "Value":
            if parent_name == "Volume":
                volume_db = max(-80, min(0, volume_db + delta))
                gain = 10 ** (volume_db / 20.0)
                write_safeload(VOLUME_ADDR, volume(volume_db),0)
                trigger_safeload()
                print(f"Volume set to {volume_db}dB (gain={gain:.6f})")
                needs_save = True
            elif parent_name == "Phase":
                phase = 1 if phase == 0 else 0  # Toggle between 0 and 1
                phase_conv = 180 if phase == 1 else 0
                write_safeload(PHASE_ADDR, phase_calc(phase_conv),0)
                trigger_safeload()
                print(f"Phase set to {'180 deg' if phase == 180 else '0 deg'}")
                needs_save = True
            elif parent_name == "Pre Gain":
                print("staattista gainia...")
                gain_db = max(-80, min(0, gain_db + delta))
                write_safeload(GAIN_ADDR, volume(gain_db),0)
                trigger_safeload()
                needs_save = True
        elif item["name"] == "Freq":
            if parent_name == "Cutoff":
                lp_freq = max(40, min(150, lp_freq + delta))
                b0, b1, b2, a1, a2 = linkwitz_riley_lowpass(lp_freq, FS)
                biquad_coeffs = coeffs_to_array(b0, b1, b2, a1, a2)
                presets[current_preset]["lp_biquads"] = [(b0, b1, b2, a1, a2), (b0, b1, b2, a1, a2)]
                for i in range(5):
                    write_safeload(LOWPASS_ADDR_1, biquad_coeffs[i], i)
                trigger_safeload()
                for i in range(5):
                    write_safeload(LOWPASS_ADDR_2, biquad_coeffs[i], i)
                trigger_safeload()
                print(f"LP Biquad: b0={b0:.6f} b1={b1:.6f} b2={b2:.6f} a1={a1:.6f} a2={a2:.6f}")
                needs_save = True
            elif parent_name == "Subsonic":
                hp_freq = max(10, min(40, hp_freq + delta))
                b0_1, b1_1, b2_1, a1_1, a2_1 = butterworth_subsonic_24(hp_freq, FS, 1)
                b0_2, b1_2, b2_2, a1_2, a2_2 = butterworth_subsonic_24(hp_freq, FS, 0)
                biquad_coeffs_1 = coeffs_to_array(b0_1, b1_1, b2_1, a1_1, a2_1)
                biquad_coeffs_2 = coeffs_to_array(b0_2, b1_2, b2_2, a1_2, a2_2)
                presets[current_preset]["hp_biquads"] = [(b0_1, b1_1, b2_1, a1_1, a2_1), (b0_2, b1_2, b2_2, a1_2, a2_2)]
                #presets[current_preset]["hp_biquads"] = [(b0_1, b1_1, b2_1, a1_1, a2_1), (b0_2, b1_2, b2_2, a1_2, a2_2)]
                for i in range(5):
                    write_safeload(SUBSONIC_ADDR_1, biquad_coeffs_1[i], i)
                trigger_safeload()
                for i in range(5):
                    write_safeload(SUBSONIC_ADDR_2, biquad_coeffs_2[i], i)
                trigger_safeload()
                print(f"HP Biquad1: b0={b0_1:.6f} b1={b1_1:.6f} b2={b2_1:.6f} a1={a1_1:.6f} a2={a2_1:.6f}")
                print(f"HP Biquad2: b0={b0_2:.6f} b1={b1_2:.6f} b2={b2_2:.6f} a1={a1_2:.6f} a2={a2_2:.6f}")
                needs_save = True
            elif parent_name.startswith("EQ"):
                eq_idx = menu_stack[-2][1]
                eq_settings[eq_idx]["freq"] = max(10, min(150, eq_settings[eq_idx]["freq"] + delta))
                b0, b1, b2, a1, a2 = parametric_eq(eq_settings[eq_idx]["freq"], eq_settings[eq_idx]["q"], eq_settings[eq_idx]["boost"])
                biquad_coeffs = coeffs_to_array(b0, b1, b2, a1, a2)
                #print("saatana", biquad_coeffs)
                presets[current_preset]["eq_biquads"][eq_idx] = (b0, b1, b2, a1, a2)
                for i in range(5):
                    write_safeload(PARAM_EQ_ADDR[eq_idx], biquad_coeffs[i], i)
                trigger_safeload()
                print(f"EQ{eq_idx+1}: b0={b0:.6f} b1={b1:.6f} b2={b2:.6f} a1={a1:.6f} a2={a2:.6f}")
                needs_save = True
        elif item["name"] == "Q":
            eq_idx = menu_stack[-2][1]
            eq_settings[eq_idx]["q"] = round(max(0.1, min(10.0, eq_settings[eq_idx]["q"] + delta * 0.1)),1)
            b0, b1, b2, a1, a2 = parametric_eq(eq_settings[eq_idx]["freq"], eq_settings[eq_idx]["q"], eq_settings[eq_idx]["boost"])
            biquad_coeffs = coeffs_to_array(b0, b1, b2, a1, a2)
            presets[current_preset]["eq_biquads"][eq_idx] = (b0, b1, b2, a1, a2)
            for i in range(5):
                write_safeload(PARAM_EQ_ADDR[eq_idx], biquad_coeffs[i], i)
            trigger_safeload()
            print(f"EQ{eq_idx+1}: b0={b0:.6f} b1={b1:.6f} b2={b2:.6f} a1={a1:.6f} a2={a2:.6f}")
            needs_save = True
        elif item["name"] == "Boost":
            eq_idx = menu_stack[-2][1]
            eq_settings[eq_idx]["boost"] = round(max(-12, min(12, eq_settings[eq_idx]["boost"] + delta * 0.1)),1)
            b0, b1, b2, a1, a2 = parametric_eq(round(eq_settings[eq_idx]["freq"]), round(eq_settings[eq_idx]["q"],1), eq_settings[eq_idx]["boost"])
            biquad_coeffs = coeffs_to_array(b0, b1, b2, a1, a2)
            print(eq_settings[eq_idx]["freq"], eq_settings[eq_idx]["q"], eq_settings[eq_idx]["boost"])
            print(b0, b1, b2, a1, a2)
            presets[current_preset]["eq_biquads"][eq_idx] = (b0, b1, b2, a1, a2)
            for i in range(5):
                write_safeload(PARAM_EQ_ADDR[eq_idx], biquad_coeffs[i], i)
            trigger_safeload()
            print(f"EQ{eq_idx+1}: b0={b0:.6f} b1={b1:.6f} b2={b2:.6f} a1={a1:.6f} a2={a2:.6f}")
            needs_save = True
        # Save updated settings to current preset
        presets[current_preset].update({
            "volume_db": volume_db,
            "phase": phase,
            "gain_db": gain_db,
            "lp_freq": lp_freq,
            "hp_freq": hp_freq,
            "eq_settings": eq_settings,
            "lp_biquads": presets[current_preset]["lp_biquads"],
            "hp_biquads": presets[current_preset]["hp_biquads"],
            "eq_biquads": presets[current_preset]["eq_biquads"]
        })
    else:
        new_cursor = (cursor + delta) % len(current_menu)
        if new_cursor < scroll_offset:
            scroll_offset = new_cursor
        elif new_cursor >= scroll_offset + MAX_VISIBLE:
            scroll_offset = new_cursor - MAX_VISIBLE + 1
        menu_stack[-1] = (current_menu, new_cursor, scroll_offset)
    display_menu()    
 
# === Main Loop ===
display_menu()
while True:
    handle_rotary()
    # Check if save is needed and delay has passed
    if needs_save and time.ticks_diff(time.ticks_ms(), last_save_time) > SAVE_DELAY_MS:
        save_presets()
    handle_oled_timeout()
    time.sleep_ms(50)