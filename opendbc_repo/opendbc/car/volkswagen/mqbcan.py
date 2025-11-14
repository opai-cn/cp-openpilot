def create_steering_control(packer, bus, apply_torque, lkas_enabled):
  values = {
    "HCA_01_Status_HCA": 5 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(apply_torque),
    "HCA_01_LM_OffSign": 1 if apply_torque < 0 else 0,
    "HCA_01_Vib_Freq": 18,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_eps_update(packer, bus, eps_stock_values, ea_simulated_torque):
  values = {s: eps_stock_values[s] for s in [
    "COUNTER",                     # Sync counter value to EPS output
    "EPS_Lenkungstyp",             # EPS rack type
    "EPS_Berechneter_LW",          # Absolute raw steering angle
    "EPS_VZ_BLW",                  # Raw steering angle sign
    "EPS_HCA_Status",              # EPS HCA control status
  ]}

  values.update({
    # Absolute driver torque input and sign, with EA inactivity mitigation
    "EPS_Lenkmoment": abs(ea_simulated_torque),
    "EPS_VZ_Lenkmoment": 1 if ea_simulated_torque < 0 else 0,
  })

  return packer.make_can_msg("LH_EPS_03", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control):
  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",   # Blind spot in warning mode on left side due to lane departure
      "LDW_SW_Warnung_rechts",  # Blind spot in warning mode on right side due to lane departure
      "LDW_Seite_DLCTLC",       # Direction of most likely lane departure (left or right)
      "LDW_DLC",                # Lane departure, distance to line crossing
      "LDW_TLC",                # Lane departure, time to line crossing
    ]}

  values.update({
    "LDW_Status_LED_gelb": 1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalter",           # ACC button, on/off
    "GRA_Typ_Hauptschalter",       # ACC main button type
    "GRA_Codierung",               # ACC button configuration/coding
    "GRA_Tip_Stufe_2",             # unknown related to stalk type
    "GRA_ButtonTypeInfo",          # unknown related to stalk type
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel,
    "GRA_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("GRA_ACC_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    acc_control = 6
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  commands = []

  acc_06_values = {
    "ACC_Typ": acc_type,
    "ACC_Status_ACC": acc_control,
    "ACC_StartStopp_Info": acc_enabled,
    "ACC_Sollbeschleunigung_02": accel if acc_enabled else 3.01,
    "ACC_zul_Regelabw_unten": 0.2,  # TODO: dynamic adjustment of comfort-band
    "ACC_zul_Regelabw_oben": 0.2,  # TODO: dynamic adjustment of comfort-band
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
  }
  commands.append(packer.make_can_msg("ACC_06", bus, acc_06_values))

  if starting:
    acc_hold_type = 4  # hold release / startup
  elif esp_hold:
    acc_hold_type = 3  # hold standby
  elif stopping:
    acc_hold_type = 1  # hold request
  else:
    acc_hold_type = 0

  acc_07_values = {
    "ACC_Anhalteweg": 0.3 if stopping else 20.46,  # Distance to stop (stopping coordinator handles terminal roll-out)
    "ACC_Freilauf_Info": 2 if acc_enabled else 0,
    "ACC_Folgebeschl": 3.02,  # Not using secondary controller accel unless and until we understand its impact
    "ACC_Sollbeschleunigung_02": accel if acc_enabled else 3.01,
    "ACC_Anforderung_HMS": acc_hold_type,
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
  }
  commands.append(packer.make_can_msg("ACC_07", bus, acc_07_values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": distance + 2,
    "ACC_Display_Prio": 3,
    "ACC_Abstandsindex": lead_distance,
  }

  return packer.make_can_msg("ACC_02", bus, values)


# AWV = Stopping Distance Reduction
# Refer to Self Study Program 890253: Volkswagen Driver Assistance Systems, Design and Function


def create_aeb_control(packer, fcw_active, aeb_active, accel):
  values = {
    "AWV_Vorstufe": 0,  # Preliminary stage
    "AWV1_Anf_Prefill": 0,  # Brake pre-fill request
    "AWV1_HBA_Param": 0,  # Brake pre-fill level
    "AWV2_Freigabe": 0,  # Stage 2 braking release
    "AWV2_Ruckprofil": 0,  # Brake jerk level
    "AWV2_Priowarnung": 0,  # Suppress lane departure warning in favor of FCW
    "ANB_Notfallblinken": 0, # Hazard flashers request
    "ANB_Teilbremsung_Freigabe": 0,  # Target braking release
    "ANB_Zielbremsung_Freigabe": 0,  # Partial braking release
    "ANB_Zielbrems_Teilbrems_Verz_Anf": 0.0,   # Acceleration requirement for target/partial braking, m/s/s
    "AWV_Halten": 0,  # Vehicle standstill request
    "PCF_Time_to_collision": 0xFF,  # Pre Crash Front, populated only with a target, might be used on Audi only
  }

  return packer.make_can_msg("ACC_10", 0, values)


def create_aeb_hud(packer, aeb_supported, fcw_active):
  values = {
    "AWV_Texte": 5 if aeb_supported else 7,  # FCW/AEB system status, display text (from menu in VAL)
    "AWV_Status_Anzeige": 1 if aeb_supported else 2,  #  FCW/AEB system status, available or disabled
  }

  return packer.make_can_msg("ACC_15", 0, values)


def volkswagen_mqb_meb_checksum(address: int, sig, d: bytearray) -> int:
  """
  Calculate CRC-8H2F/AUTOSAR checksum for MQB/MEB platform CAN messages.
  This is the same algorithm used in safety_volkswagen_common.h
  """
  # CRC-8H2F polynomial (0x2F)
  crc8_lut_8h2f = [
    0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A,
    0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63, 0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34,
    0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
    0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47,
    0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B, 0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C,
    0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
    0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F,
    0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6, 0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1,
    0xE3, 0xCC, 0xBD, 0x92, 0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
    0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7,
    0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D, 0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A,
    0x3E, 0x11, 0x60, 0x4F, 0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
    0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23, 0xEE, 0xC1, 0xB0, 0x9F,
    0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66, 0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31,
    0x76, 0x59, 0x28, 0x07, 0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
    0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42
  ]

  crc = 0xFF
  # Process all bytes except the first (checksum byte)
  for i in range(1, len(d)):
    crc ^= d[i]
    crc = crc8_lut_8h2f[crc]

  # Get counter from byte 1, bits 0-3
  counter = d[1] & 0x0F

  # Apply address-specific XOR based on counter
  if address == 0x09F:  # MSG_LH_EPS_03
    crc ^= 0xF5
  elif address == 0x106:  # MSG_ESP_05
    crc ^= 0x07
  elif address == 0x120:  # MSG_TSK_06
    xor_table = [0xC4, 0xE2, 0x4F, 0xE4, 0xF8, 0x2F, 0x56, 0x81, 0x9F, 0xE5, 0x83, 0x44, 0x05, 0x3F, 0x97, 0xDF]
    crc ^= xor_table[counter]
  elif address == 0x121:  # MSG_MOTOR_20
    xor_table = [0xE9, 0x65, 0xAE, 0x6B, 0x7B, 0x35, 0xE5, 0x5F, 0x4E, 0xC7, 0x86, 0xA2, 0xBB, 0xDD, 0xEB, 0xB4]
    crc ^= xor_table[counter]
  elif address == 0x12B:  # MSG_GRA_ACC_01
    xor_table = [0x6A, 0x38, 0xB4, 0x27, 0x22, 0xEF, 0xE1, 0xBB, 0xF8, 0x80, 0x84, 0x49, 0xC7, 0x9E, 0x1E, 0x2B]
    crc ^= xor_table[counter]

  crc = crc8_lut_8h2f[crc]
  return crc ^ 0xFF


def xor_checksum(address: int, sig, d: bytearray) -> int:
  """
  Simple XOR checksum for Volkswagen messages.
  XOR all bytes together.
  """
  checksum = 0
  for byte in d:
    checksum ^= byte
  return checksum
