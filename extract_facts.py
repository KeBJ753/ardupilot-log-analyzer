#!/usr/bin/env python3
"""
extract_facts.py
=================
Extrae HECHOS VERIFICABLES de un log .BIN de ArduPilot/ArduPlane.
NO interpreta, NO redacta, NO usa IA. Solo calcula numeros exactos
y los guarda en un JSON estructurado.

Este JSON es la UNICA fuente de verdad que se le pasa despues a la IA
(GPT, Gemini, etc.) para que redacte el informe. La IA nunca debe
inventar un numero que no este aqui.

Uso:
    python extract_facts.py archivo.BIN
    -> genera archivo.facts.json

Requiere: pip install pymavlink
"""

import sys
import json
import os
from pymavlink import mavutil

# ────────────────────────────────────────────────────────────────────
# CONFIGURACION DE UMBRALES (ajustar segun el sistema real del labo)
# ────────────────────────────────────────────────────────────────────

THRESHOLDS = {
    "gps_hdop_excelente": 1.5,
    "gps_hdop_critico": 2.5,
    "vibe_normal": 15.0,
    "vibe_critico": 30.0,
    "att_roll_critico_fw": 70.0,      # grados, ala fija
    "att_pitch_critico_fw": 50.0,     # grados, ala fija
    "att_roll_limite_normal": 45.0,   # grados, limite tipico LIM_ROLL_CD
    "att_pitch_limite_normal": 35.0,
    "actitud_error_excelente": 2.0,   # grados
    "actitud_error_bueno": 5.0,
    "actitud_error_aceptable": 10.0,
    "log_gap_segundos": 30,           # gap que se considera relevante
    "lipo_celda_full": 4.2,
    "lipo_celda_nominal": 3.7,
    "lipo_celda_min_seguro": 3.5,
    "lipo_celda_critico": 3.3,
    "lipo_celda_dano": 3.2,
}

# Especificaciones de bateria conocidas - EDITAR SEGUN EL DRON REAL
# Si no se conoce, dejar en None y el script reportara "no verificado"
BATTERY_SPEC = {
    "celdas": None,        # ej: 10  (10S)
    "capacidad_mah": None, # ej: 14400
    "quimica": "LiPo",
}

MODE_NAMES = {
    0:'MANUAL', 1:'CIRCLE', 2:'STABILIZE', 3:'TRAINING', 4:'ACRO',
    5:'FLY_BY_WIRE_A', 6:'FLY_BY_WIRE_B', 7:'CRUISE', 8:'AUTOTUNE',
    10:'AUTO', 11:'RTL', 12:'LOITER', 13:'TAKEOFF', 14:'AVOID_ADSB',
    15:'GUIDED', 17:'QSTABILIZE', 18:'QHOVER', 19:'QLOITER',
    20:'QLAND', 21:'QRTL', 22:'QAUTOTUNE', 23:'QACRO', 24:'THERMAL',
    25:'LOITER_TO_ALT'
}

SUBSYS_NAMES = {
    1:'Main', 2:'Radio', 3:'Compass', 4:'OptFlow', 5:'FailSafe_Radio',
    6:'FailSafe_Batt', 7:'FailSafe_GPS', 8:'FailSafe_GCS',
    9:'FailSafe_Fence', 10:'Flight_Mode', 11:'GPS', 12:'Crash',
    13:'EKF_Check', 14:'FailSafe_EKF', 15:'Barometer', 16:'CPU',
    17:'Radio_Version', 18:'FailSafe_ADSB', 19:'Timer', 20:'Baro_Glitch'
}


def safe_float(val):
    """Convierte a float si es posible, si no devuelve None."""
    try:
        if val is None:
            return None
        f = float(val)
        if f != f:  # NaN check
            return None
        return f
    except (ValueError, TypeError):
        return None


def percentile(sorted_list, pct):
    """Percentil simple sobre lista ya ordenada."""
    if not sorted_list:
        return None
    idx = int(len(sorted_list) * pct)
    idx = min(idx, len(sorted_list) - 1)
    return sorted_list[idx]


def extract_facts(bin_path):
    """
    Extrae todos los hechos del log. Cada seccion del dict de salida
    corresponde a una seccion del informe final.
    """
    log = mavutil.mavlink_connection(bin_path, robust_parsing=True)

    # Acumuladores
    msgs = []
    errors = []
    modes = []
    att_rows = []
    gps_rows = []
    curr_rows = []
    baro_rows = []
    vibe_rows = []
    tecs_rows = []
    ctun_rows = []
    all_timestamps = []
    flight_relevant_timestamps = []  # ATT/CTUN/GPS - NO IMU (que nunca para)

    first_ts = None
    last_ts = None
    firmware_info = {}

    while True:
        m = log.recv_match(blocking=False)
        if m is None:
            break
        mtype = m.get_type()
        ts = getattr(m, 'TimeUS', None)

        if ts:
            if first_ts is None:
                first_ts = ts
            last_ts = ts
            all_timestamps.append(ts)
            # ATT/CTUN/GPS/CURR son indicadores reales de actividad de vuelo.
            # IMU/RATE/etc. se siguen grabando en tierra y NUNCA paran,
            # asi que no sirven para detectar gaps de vuelo activo.
            if mtype in ('ATT', 'CTUN', 'NTUN', 'GPS', 'CURR', 'TECS'):
                flight_relevant_timestamps.append(ts)

        if mtype == 'MSG':
            text = getattr(m, 'Message', '')
            msgs.append({"t_us": ts, "text": text})
            # Detectar firmware/hardware del texto (sin asumir nada mas)
            if 'ArduPlane' in text or 'ArduCopter' in text:
                firmware_info['firmware'] = text
            if 'CubeOrange' in text or 'CubeBlack' in text or 'CubeYellow' in text:
                firmware_info['hardware'] = text
            if 'QuadPlane Frame' in text:
                firmware_info['frame'] = text
            if 'RC Protocol' in text:
                firmware_info['rc_protocol'] = text

        if mtype == 'ERR':
            errors.append({
                "t_us": ts,
                "subsys_code": int(getattr(m, 'Subsys', -1)),
                "subsys_name": SUBSYS_NAMES.get(int(getattr(m, 'Subsys', -1)), f"unknown_{getattr(m,'Subsys','?')}"),
                "ecode": int(getattr(m, 'ECode', -1)),
            })

        if mtype == 'MODE':
            mode_num = int(getattr(m, 'Mode', -1))
            modes.append({
                "t_us": ts,
                "mode_num": mode_num,
                "mode_name": MODE_NAMES.get(mode_num, f"mode_{mode_num}")
            })

        if mtype == 'ATT':
            att_rows.append({
                "t_us": ts,
                "roll": safe_float(getattr(m, 'Roll', None)),
                "pitch": safe_float(getattr(m, 'Pitch', None)),
                "yaw": safe_float(getattr(m, 'Yaw', None)),
                "des_roll": safe_float(getattr(m, 'DesRoll', None)),
                "des_pitch": safe_float(getattr(m, 'DesPitch', None)),
            })

        if mtype == 'GPS':
            gps_rows.append({
                "t_us": ts,
                "status": safe_float(getattr(m, 'Status', None)),
                "nsats": safe_float(getattr(m, 'NSats', None)),
                "hdop": safe_float(getattr(m, 'HDop', None)),
                "lat": safe_float(getattr(m, 'Lat', None)),
                "lng": safe_float(getattr(m, 'Lng', None)),
                "alt": safe_float(getattr(m, 'Alt', None)),
                "spd": safe_float(getattr(m, 'Spd', None)),
            })

        if mtype == 'CURR':
            curr_rows.append({
                "t_us": ts,
                "volt": safe_float(getattr(m, 'Volt', None)),
                "curr": safe_float(getattr(m, 'Curr', None)),
                "curr_tot": safe_float(getattr(m, 'CurrTot', None)),
            })

        if mtype == 'BARO':
            baro_rows.append({
                "t_us": ts,
                "alt": safe_float(getattr(m, 'Alt', None)),
                "temp": safe_float(getattr(m, 'Temp', None)),
            })

        if mtype == 'VIBE':
            vibe_rows.append({
                "t_us": ts,
                "vibe_x": safe_float(getattr(m, 'VibeX', None)),
                "vibe_y": safe_float(getattr(m, 'VibeY', None)),
                "vibe_z": safe_float(getattr(m, 'VibeZ', None)),
                "clip0": safe_float(getattr(m, 'Clip0', None)),
                "clip1": safe_float(getattr(m, 'Clip1', None)),
                "clip2": safe_float(getattr(m, 'Clip2', None)),
            })

        if mtype == 'TECS':
            tecs_rows.append({
                "t_us": ts,
                "airspeed_demand": safe_float(getattr(m, 'Spdem', None)),
                "airspeed_actual": safe_float(getattr(m, 'Sp', None)),
                "alt_demand": safe_float(getattr(m, 'Hdem', None)),
                "alt_actual": safe_float(getattr(m, 'H', None)),
            })

        if mtype == 'CTUN':
            ctun_rows.append({
                "t_us": ts,
                "alt": safe_float(getattr(m, 'Alt', None)),
                "talt": safe_float(getattr(m, 'TAlt', None)),
                "thr_out": safe_float(getattr(m, 'ThrOut', None)),
            })

    # ── Si no hay timestamps, el archivo esta vacio o corrupto ──
    if first_ts is None:
        return {
            "error": "NO_TIMESTAMPS_FOUND",
            "message": "El archivo no contiene mensajes con TimeUS. Puede estar corrupto o vacio."
        }

    duration_s = (last_ts - first_ts) / 1e6

    def t_rel(ts_us):
        """Convierte timestamp absoluto a segundos relativos al inicio."""
        if ts_us is None:
            return None
        return round((ts_us - first_ts) / 1e6, 3)

    # ════════════════════════════════════════════════════════════
    # SECCION 1: METADATA Y DURACION
    # ════════════════════════════════════════════════════════════
    facts = {
        "_meta": {
            "archivo": os.path.basename(bin_path),
            "umbrales_usados": THRESHOLDS,
            "advertencia": "Todos los valores aqui son calculados directamente del log. Ningun valor fue inferido o asumido por una IA."
        },
        "resumen_general": {
            "duracion_total_s": round(duration_s, 1),
            "duracion_total_min": round(duration_s / 60, 2),
            "timestamp_inicio_us": first_ts,
            "timestamp_fin_us": last_ts,
            "firmware": firmware_info.get('firmware', "NO_DETECTADO"),
            "hardware": firmware_info.get('hardware', "NO_DETECTADO"),
            "frame": firmware_info.get('frame', "NO_DETECTADO"),
            "rc_protocol": firmware_info.get('rc_protocol', "NO_DETECTADO"),
            "total_mensajes_msg": len(msgs),
            "total_errores_err": len(errors),
            "total_cambios_modo": len(modes),
        }
    }

    # ════════════════════════════════════════════════════════════
    # SECCION 2: GAPS EN MENSAJES RELEVANTES DE VUELO (ATT/CTUN/GPS/CURR/TECS)
    # NOTA IMPORTANTE: los mensajes IMU/RATE de alta frecuencia NUNCA
    # se detienen, incluso con el dron desarmado en tierra. Por eso
    # esta deteccion usa SOLO mensajes que indican actividad de vuelo
    # real (actitud, navegacion, GPS, energia). Un gap aqui significa
    # que el sistema dejo de recibir/calcular estos datos -- consistente
    # con un crash, reboot, o perdida de EKF -- no solo con "estar en tierra".
    # ════════════════════════════════════════════════════════════
    gaps = []
    sorted_ts = sorted(flight_relevant_timestamps)
    for i in range(1, len(sorted_ts)):
        gap_s = (sorted_ts[i] - sorted_ts[i-1]) / 1e6
        if gap_s > THRESHOLDS["log_gap_segundos"]:
            gaps.append({
                "inicio_t_s": t_rel(sorted_ts[i-1]),
                "fin_t_s": t_rel(sorted_ts[i]),
                "duracion_s": round(gap_s, 1)
            })
    facts["gaps_en_log"] = gaps
    facts["resumen_general"]["nota_metodo_gaps"] = (
        "Los gaps se calculan sobre mensajes ATT/CTUN/NTUN/GPS/CURR/TECS, "
        "no sobre IMU. IMU se sigue grabando en tierra sin parar, por lo "
        "que no es util para detectar el fin del vuelo activo."
    )
    facts["resumen_general"]["tiene_gaps_sospechosos"] = len(gaps) > 0
    if gaps:
        # El "vuelo activo" termina probablemente en el primer gap grande
        facts["resumen_general"]["fin_vuelo_activo_estimado_s"] = gaps[0]["inicio_t_s"]

    # ════════════════════════════════════════════════════════════
    # SECCION 3: MODOS DE VUELO (linea de tiempo completa)
    # ════════════════════════════════════════════════════════════
    modes_timeline = []
    for i, mode_event in enumerate(modes):
        start_s = t_rel(mode_event["t_us"])
        end_s = t_rel(modes[i+1]["t_us"]) if i+1 < len(modes) else round(duration_s, 1)
        modes_timeline.append({
            "modo": mode_event["mode_name"],
            "modo_num": mode_event["mode_num"],
            "inicio_s": start_s,
            "fin_s": end_s,
            "duracion_s": round(end_s - start_s, 1) if start_s is not None else None
        })
    facts["modos_de_vuelo"] = modes_timeline

    # Conteo de transiciones rapidas (posible indicador de inestabilidad)
    transiciones_rapidas = sum(
        1 for m in modes_timeline
        if m["duracion_s"] is not None and m["duracion_s"] < 5
        and m["modo"] not in ("MANUAL",)
    )
    facts["resumen_general"]["transiciones_modo_menores_5s"] = transiciones_rapidas

    # ════════════════════════════════════════════════════════════
    # SECCION 4: ERRORES (lista completa, sin interpretar)
    # ════════════════════════════════════════════════════════════
    errors_out = []
    for e in errors:
        errors_out.append({
            "t_s": t_rel(e["t_us"]),
            "subsistema": e["subsys_name"],
            "subsistema_codigo": e["subsys_code"],
            "codigo_error": e["ecode"],
        })
    facts["errores"] = errors_out

    # Mensajes de texto relevantes (failsafes, armados, etc.)
    keywords_criticos = [
        'failsafe', 'Failsafe', 'FAILSAFE', 'disarm', 'armed',
        'EKF', 'Land complete', 'Overshoot', 'PreArm', 'GPS',
        'Crash', 'crash'
    ]
    msgs_relevantes = []
    for msg in msgs:
        if any(kw in msg["text"] for kw in keywords_criticos):
            msgs_relevantes.append({
                "t_s": t_rel(msg["t_us"]),
                "texto": msg["text"]
            })
    facts["mensajes_relevantes"] = msgs_relevantes
    facts["mensajes_todos"] = [{"t_s": t_rel(m["t_us"]), "texto": m["text"]} for m in msgs]

    # Eventos criticos que aparecen como TEXTO en MSG, no como ERR
    # estructurado. ArduPilot reporta varios fallos importantes asi
    # (failsafes de RC, EKF deteniendo aiding, etc.) por lo que deben
    # buscarse explicitamente en el texto, no asumir que "errores" los
    # captura todos.
    patrones_texto_criticos = {
        "rc_failsafe": ['Throttle failsafe', 'RC Failsafe', 'FS_THR'],
        "ekf_stopped_aiding": ['EKF3 stopped', 'stopped aiding', 'EKF2 stopped'],
        "ekf_lane_switch": ['EKF3 lane switch', 'lane switch'],
        "crash_detectado_por_firmware": ['Crash:', 'CRASH'],
        "gps_glitch": ['GPS Glitch', 'GPS 1: error changed'],
    }
    eventos_texto_criticos = {}
    for categoria, patrones in patrones_texto_criticos.items():
        encontrados = []
        for msg in msgs:
            if any(p in msg["text"] for p in patrones):
                encontrados.append({"t_s": t_rel(msg["t_us"]), "texto": msg["text"]})
        eventos_texto_criticos[categoria] = encontrados
    facts["eventos_criticos_en_texto"] = eventos_texto_criticos
    facts["resumen_general"]["nota_eventos_texto"] = (
        "ArduPilot reporta varios fallos importantes (RC failsafe, EKF "
        "stopped aiding, etc.) como texto libre en mensajes MSG, no como "
        "ERR estructurado. El campo 'errores' de este JSON NO los incluye. "
        "Revisar 'eventos_criticos_en_texto' para esos casos."
    )

    # ════════════════════════════════════════════════════════════
    # SECCION 5: GPS
    # ════════════════════════════════════════════════════════════
    if gps_rows:
        statuses = [r["status"] for r in gps_rows if r["status"] is not None]
        hdops = [r["hdop"] for r in gps_rows if r["hdop"] is not None]
        nsats = [r["nsats"] for r in gps_rows if r["nsats"] is not None]
        valid_count = sum(1 for s in statuses if s >= 3)
        total_count = len(statuses)

        # Errores GPS especificos (subsistema 11)
        gps_errs = [e for e in errors if e["subsys_code"] == 11]

        facts["gps"] = {
            "muestras_totales": total_count,
            "muestras_con_fix_valido": valid_count,
            "porcentaje_disponibilidad": round(valid_count / total_count * 100, 1) if total_count else None,
            "hdop_promedio": round(sum(hdops)/len(hdops), 3) if hdops else None,
            "hdop_minimo": round(min(hdops), 3) if hdops else None,
            "hdop_maximo": round(max(hdops), 3) if hdops else None,
            "satelites_minimo": min(nsats) if nsats else None,
            "satelites_maximo": max(nsats) if nsats else None,
            "eventos_error_gps_subsistema": len(gps_errs),
            "lat_inicial": gps_rows[0]["lat"],
            "lng_inicial": gps_rows[0]["lng"],
            "alt_gps_minima": min((r["alt"] for r in gps_rows if r["alt"]), default=None),
            "alt_gps_maxima": max((r["alt"] for r in gps_rows if r["alt"]), default=None),
            "velocidad_gps_maxima": max((r["spd"] for r in gps_rows if r["spd"] is not None), default=None),
            "evaluacion_automatica": (
                "EXCELENTE" if (valid_count/total_count*100 if total_count else 0) >= 99 else
                "ACEPTABLE" if (valid_count/total_count*100 if total_count else 0) >= 90 else
                "DEFICIENTE"
            )
        }
    else:
        facts["gps"] = {"disponible": False, "nota": "No hay mensajes GPS en este log"}

    # ════════════════════════════════════════════════════════════
    # SECCION 6: BAROMETRO / ALTITUD
    # ════════════════════════════════════════════════════════════
    if baro_rows:
        alts = [r["alt"] for r in baro_rows if r["alt"] is not None]
        alt_inicial = alts[0] if alts else None
        max_alt_relativa = max((abs(a - alt_inicial) for a in alts), default=0) if alt_inicial is not None else None

        # IMPORTANTE: un solo spike puntual (comun al energizar el
        # barometro) NO cuenta como vuelo real. Exigimos que el cambio
        # de altitud sea SOSTENIDO durante al menos N muestras
        # consecutivas para descartar transitorios del sensor.
        VENTANA_SOSTENIDA = 10  # muestras consecutivas
        UMBRAL_VUELO_M = 5.0
        vuelo_sostenido = False
        if alt_inicial is not None and len(alts) >= VENTANA_SOSTENIDA:
            for i in range(len(alts) - VENTANA_SOSTENIDA):
                ventana = alts[i:i+VENTANA_SOSTENIDA]
                if all(abs(a - alt_inicial) > UMBRAL_VUELO_M for a in ventana):
                    vuelo_sostenido = True
                    break

        facts["altitud_barometrica"] = {
            "alt_minima_m": round(min(alts), 2) if alts else None,
            "alt_maxima_m": round(max(alts), 2) if alts else None,
            "alt_inicial_m": round(alt_inicial, 2) if alt_inicial is not None else None,
            "alt_final_m": round(alts[-1], 2) if alts else None,
            "cambio_max_relativo_m": round(max_alt_relativa, 2) if max_alt_relativa is not None else None,
            "se_detecto_vuelo_real": vuelo_sostenido,
            "nota": (
                f"se_detecto_vuelo_real=True solo si el cambio de altitud "
                f"relativa supera {UMBRAL_VUELO_M}m de forma SOSTENIDA durante "
                f"al menos {VENTANA_SOSTENIDA} muestras consecutivas (no un "
                f"solo spike puntual, que puede ser ruido transitorio del "
                f"sensor al energizarse)."
            )
        }
    else:
        facts["altitud_barometrica"] = {"disponible": False}

    # ════════════════════════════════════════════════════════════
    # SECCION 7: ACTITUD (ATT)
    # ════════════════════════════════════════════════════════════
    if att_rows:
        rolls = [r["roll"] for r in att_rows if r["roll"] is not None]
        pitches = [r["pitch"] for r in att_rows if r["pitch"] is not None]

        roll_errors = [abs(r["roll"]-r["des_roll"]) for r in att_rows
                       if r["roll"] is not None and r["des_roll"] is not None]
        pitch_errors = [abs(r["pitch"]-r["des_pitch"]) for r in att_rows
                         if r["pitch"] is not None and r["des_pitch"] is not None]

        roll_errors_sorted = sorted(roll_errors)
        pitch_errors_sorted = sorted(pitch_errors)

        # Muestras criticas: roll o pitch fuera de umbral critico
        criticas = []
        for r in att_rows:
            if r["roll"] is not None and r["pitch"] is not None:
                if abs(r["roll"]) > THRESHOLDS["att_roll_critico_fw"] or abs(r["pitch"]) > THRESHOLDS["att_pitch_critico_fw"]:
                    criticas.append({
                        "t_s": t_rel(r["t_us"]),
                        "roll": round(r["roll"], 1),
                        "pitch": round(r["pitch"], 1)
                    })

        facts["actitud"] = {
            "roll_minimo": round(min(rolls), 2) if rolls else None,
            "roll_maximo": round(max(rolls), 2) if rolls else None,
            "pitch_minimo": round(min(pitches), 2) if pitches else None,
            "pitch_maximo": round(max(pitches), 2) if pitches else None,
            "error_roll_promedio": round(sum(roll_errors)/len(roll_errors), 3) if roll_errors else None,
            "error_roll_maximo": round(max(roll_errors), 3) if roll_errors else None,
            "error_roll_p95": round(percentile(roll_errors_sorted, 0.95), 3) if roll_errors_sorted else None,
            "error_pitch_promedio": round(sum(pitch_errors)/len(pitch_errors), 3) if pitch_errors else None,
            "error_pitch_maximo": round(max(pitch_errors), 3) if pitch_errors else None,
            "error_pitch_p95": round(percentile(pitch_errors_sorted, 0.95), 3) if pitch_errors_sorted else None,
            "muestras_criticas": criticas,
            "total_muestras_criticas": len(criticas),
            "umbral_critico_roll": THRESHOLDS["att_roll_critico_fw"],
            "umbral_critico_pitch": THRESHOLDS["att_pitch_critico_fw"],
        }
    else:
        facts["actitud"] = {"disponible": False}

    # ════════════════════════════════════════════════════════════
    # SECCION 8: VIBRACION (VIBE)
    # ════════════════════════════════════════════════════════════
    if vibe_rows:
        vx = [r["vibe_x"] for r in vibe_rows if r["vibe_x"] is not None]
        vy = [r["vibe_y"] for r in vibe_rows if r["vibe_y"] is not None]
        vz = [r["vibe_z"] for r in vibe_rows if r["vibe_z"] is not None]
        c0 = [r["clip0"] for r in vibe_rows if r["clip0"] is not None]

        def vibe_eval(p95):
            if p95 is None:
                return "SIN_DATOS"
            if p95 > THRESHOLDS["vibe_critico"]:
                return "CRITICO"
            if p95 > THRESHOLDS["vibe_normal"]:
                return "PREOCUPANTE"
            return "NORMAL"

        vx_sorted, vy_sorted, vz_sorted = sorted(vx), sorted(vy), sorted(vz)
        p95x = percentile(vx_sorted, 0.95)
        p95y = percentile(vy_sorted, 0.95)
        p95z = percentile(vz_sorted, 0.95)

        facts["vibracion"] = {
            "vibe_x_promedio": round(sum(vx)/len(vx), 3) if vx else None,
            "vibe_x_maximo": round(max(vx), 3) if vx else None,
            "vibe_x_p95": round(p95x, 3) if p95x is not None else None,
            "vibe_x_evaluacion": vibe_eval(p95x),
            "vibe_y_promedio": round(sum(vy)/len(vy), 3) if vy else None,
            "vibe_y_maximo": round(max(vy), 3) if vy else None,
            "vibe_y_p95": round(p95y, 3) if p95y is not None else None,
            "vibe_y_evaluacion": vibe_eval(p95y),
            "vibe_z_promedio": round(sum(vz)/len(vz), 3) if vz else None,
            "vibe_z_maximo": round(max(vz), 3) if vz else None,
            "vibe_z_p95": round(p95z, 3) if p95z is not None else None,
            "vibe_z_evaluacion": vibe_eval(p95z),
            "clip0_maximo": max(c0) if c0 else None,
            "umbral_normal": THRESHOLDS["vibe_normal"],
            "umbral_critico": THRESHOLDS["vibe_critico"],
            "metodo": "Evaluacion basada en PERCENTIL 95, no en el maximo puntual, para evitar falsos criticos por picos momentaneos."
        }
    else:
        facts["vibracion"] = {"disponible": False}

    # ════════════════════════════════════════════════════════════
    # SECCION 9: ENERGIA (CURR / BAT)
    # ════════════════════════════════════════════════════════════
    if curr_rows:
        volts = [r["volt"] for r in curr_rows if r["volt"] is not None and r["volt"] > 0]
        currs = [r["curr"] for r in curr_rows if r["curr"] is not None and r["curr"] >= 0]
        tots = [r["curr_tot"] for r in curr_rows if r["curr_tot"] is not None]

        v_inicial = volts[0] if volts else None
        v_final = volts[-1] if volts else None

        # Verificacion contra spec de bateria SI esta configurada
        battery_check = {"spec_configurada": False}
        if BATTERY_SPEC["celdas"] and v_inicial:
            cells = BATTERY_SPEC["celdas"]
            v_full_pack = cells * THRESHOLDS["lipo_celda_full"]
            v_per_cell_inicial = v_inicial / cells
            v_per_cell_final = (v_final / cells) if v_final else None
            battery_check = {
                "spec_configurada": True,
                "celdas_configuradas": cells,
                "v_carga_completa_teorica": round(v_full_pack, 2),
                "v_por_celda_inicial": round(v_per_cell_inicial, 3),
                "v_por_celda_final": round(v_per_cell_final, 3) if v_per_cell_final else None,
                "voltaje_inicial_coherente": v_inicial <= v_full_pack * 1.02,  # 2% margen de sensor
                "alerta_voltaje_final_critico": v_per_cell_final is not None and v_per_cell_final < THRESHOLDS["lipo_celda_critico"],
            }

        # Deteccion de corriente fisicamente imposible (umbral generico, ajustar por sistema)
        curr_max = max(currs) if currs else None
        facts["energia"] = {
            "voltaje_inicial": round(v_inicial, 3) if v_inicial else None,
            "voltaje_final": round(v_final, 3) if v_final else None,
            "voltaje_minimo": round(min(volts), 3) if volts else None,
            "voltaje_maximo": round(max(volts), 3) if volts else None,
            "caida_voltaje_total": round(v_inicial - v_final, 3) if (v_inicial and v_final) else None,
            "corriente_minima": round(min(currs), 3) if currs else None,
            "corriente_maxima": round(curr_max, 3) if curr_max is not None else None,
            "corriente_promedio": round(sum(currs)/len(currs), 3) if currs else None,
            "mah_consumidos_segun_sensor": round(max(tots), 1) if tots else None,
            "battery_spec_check": battery_check,
            "advertencia_corriente_baja_sospechosa": curr_max is not None and curr_max < 5.0,
            "nota": "advertencia_corriente_baja_sospechosa=True si la corriente maxima registrada es menor a 5A, lo cual es fisicamente improbable durante vuelo activo de un QuadPlane. Puede indicar sensor sin calibrar."
        }
    else:
        facts["energia"] = {"disponible": False, "nota": "Sin mensajes CURR en este log. Verificar LOG_BITMASK."}

    # ════════════════════════════════════════════════════════════
    # SECCION 10: TECS / NAVEGACION (solo si hay datos)
    # ════════════════════════════════════════════════════════════
    if tecs_rows:
        speeds_actual = [r["airspeed_actual"] for r in tecs_rows if r["airspeed_actual"] is not None]
        speeds_demand = [r["airspeed_demand"] for r in tecs_rows if r["airspeed_demand"] is not None]
        facts["navegacion_tecs"] = {
            "airspeed_actual_min": round(min(speeds_actual), 2) if speeds_actual else None,
            "airspeed_actual_max": round(max(speeds_actual), 2) if speeds_actual else None,
            "airspeed_demand_min": round(min(speeds_demand), 2) if speeds_demand else None,
            "airspeed_demand_max": round(max(speeds_demand), 2) if speeds_demand else None,
        }
    else:
        facts["navegacion_tecs"] = {"disponible": False}

    # ════════════════════════════════════════════════════════════
    # SECCION 11: CLASIFICACION AUTOMATICA DEL TIPO DE SESION
    # ════════════════════════════════════════════════════════════
    # CRITERIO CORREGIDO: no basta con "ocurre despues del primer
    # despegue". Un aterrizaje normal seguido de espera en tierra
    # (ej. recargando una mision) es indistinguible de un crash si
    # solo miramos "antes/despues del primer t de vuelo". Por eso
    # se construye una serie de "esta en el aire" (altitud > umbral)
    # punto por punto, y un evento solo se considera sospechoso si
    # ocurre mientras el dron esta efectivamente en el aire en ese
    # instante (no despues de haber vuelto a tierra).
    vuelo_real = facts.get("altitud_barometrica", {}).get("se_detecto_vuelo_real", False)

    # Construir serie temporal de "en el aire" a partir de BARO
    en_el_aire_series = []  # lista de (t_s, bool)
    if baro_rows:
        alt_inicial_baro = baro_rows[0]["alt"]
        if alt_inicial_baro is not None:
            for r in baro_rows:
                if r["alt"] is not None:
                    en_aire = abs(r["alt"] - alt_inicial_baro) > 5.0
                    en_el_aire_series.append((t_rel(r["t_us"]), en_aire))

    def esta_en_el_aire_en(t_s):
        """
        Busca el estado 'en el aire' mas cercano a t_s (busqueda lineal
        simple; los logs no son tan grandes como para necesitar algo
        mas sofisticado, y la claridad importa mas que la velocidad aqui).
        """
        if not en_el_aire_series or t_s is None:
            return False
        mejor = None
        mejor_diff = None
        for t_ref, en_aire in en_el_aire_series:
            if t_ref is None:
                continue
            diff = abs(t_ref - t_s)
            if mejor_diff is None or diff < mejor_diff:
                mejor_diff = diff
                mejor = en_aire
        return bool(mejor)

    # Gaps sospechosos: solo si el dron estaba EN EL AIRE justo antes
    # de que el gap comenzara (un gap que empieza con el dron ya en
    # tierra es indistinguible de "esperando en tierra").
    gaps_durante_vuelo = [g for g in gaps if esta_en_el_aire_en(g["inicio_t_s"])]

    # Errores ERR sospechosos: solo si el dron estaba en el aire en
    # ese instante exacto.
    errores_durante_vuelo = [e for e in errors_out if esta_en_el_aire_en(e["t_s"])]

    # Eventos de texto criticos (failsafe, EKF stopped, crash) sospechosos
    # solo si ocurren con el dron en el aire.
    eventos_graves_durante_vuelo = []
    for categoria in ("rc_failsafe", "ekf_stopped_aiding", "crash_detectado_por_firmware"):
        for ev in eventos_texto_criticos.get(categoria, []):
            if esta_en_el_aire_en(ev["t_s"]):
                eventos_graves_durante_vuelo.append({"categoria": categoria, **ev})

    tiene_errores_criticos_en_vuelo = len(errores_durante_vuelo) > 0
    tiene_gap_sospechoso_en_vuelo = len(gaps_durante_vuelo) > 0
    tiene_evento_grave_en_vuelo = len(eventos_graves_durante_vuelo) > 0
    tiene_actitud_critica = facts.get("actitud", {}).get("total_muestras_criticas", 0) > 0

    if not vuelo_real and duration_s < 180:
        clasificacion = "ENCENDIDO_APAGADO_O_CALIBRACION"
    elif not vuelo_real:
        clasificacion = "PRUEBA_EN_TIERRA"
    elif vuelo_real and (tiene_gap_sospechoso_en_vuelo or tiene_actitud_critica or tiene_evento_grave_en_vuelo):
        clasificacion = "POSIBLE_ACCIDENTE_O_ANOMALIA_GRAVE"
    elif vuelo_real and tiene_errores_criticos_en_vuelo:
        clasificacion = "VUELO_CON_ANOMALIA"
    elif vuelo_real:
        clasificacion = "VUELO_NORMAL_O_MISION"
    else:
        clasificacion = "INDETERMINADO"

    facts["clasificacion_automatica"] = {
        "categoria": clasificacion,
        "criterios_evaluados": {
            "vuelo_real_detectado": vuelo_real,
            "tiene_errores_criticos_en_el_aire": tiene_errores_criticos_en_vuelo,
            "tiene_gap_sospechoso_en_el_aire": tiene_gap_sospechoso_en_vuelo,
            "tiene_evento_grave_en_el_aire": tiene_evento_grave_en_vuelo,
            "tiene_actitud_critica": tiene_actitud_critica,
            "duracion_s": round(duration_s, 1),
            "errores_en_tierra_ignorados_para_clasificacion": len(errors_out) - len(errores_durante_vuelo),
            "gaps_en_tierra_ignorados_para_clasificacion": len(gaps) - len(gaps_durante_vuelo),
        },
        "eventos_graves_en_el_aire": eventos_graves_durante_vuelo,
        "advertencia": (
            "Esta clasificacion es una sugerencia basada en reglas fijas. "
            "Un evento (error/gap/failsafe) solo se considera sospechoso "
            "si el dron estaba efectivamente EN EL AIRE en ese instante "
            "(altitud barometrica >5m respecto al despegue), determinado "
            "punto por punto -- no simplemente 'despues del primer despegue'. "
            "Esto evita confundir un aterrizaje normal seguido de espera en "
            "tierra con un crash. La IA que redacte el informe debe "
            "verificar esta clasificacion contra el resto de los datos, "
            "no asumirla como verdad absoluta."
        )
    }

    return facts


def main():
    if len(sys.argv) < 2:
        print("Uso: python extract_facts.py archivo.BIN [archivo2.BIN ...]")
        print("     o: python extract_facts.py /ruta/a/carpeta_con_bins/")
        sys.exit(1)

    targets = []
    for arg in sys.argv[1:]:
        if os.path.isdir(arg):
            for fname in sorted(os.listdir(arg)):
                if fname.upper().endswith('.BIN'):
                    targets.append(os.path.join(arg, fname))
        elif os.path.isfile(arg):
            targets.append(arg)
        else:
            print(f"AVISO: '{arg}' no existe, se omite.")

    if not targets:
        print("No se encontraron archivos .BIN para procesar.")
        sys.exit(1)

    print(f"Procesando {len(targets)} archivo(s)...\n")

    for bin_path in targets:
        print(f"  -> {os.path.basename(bin_path)} ...", end=" ", flush=True)
        try:
            facts = extract_facts(bin_path)
            out_path = os.path.splitext(bin_path)[0] + ".facts.json"
            with open(out_path, 'w', encoding='utf-8') as f:
                json.dump(facts, f, indent=2, ensure_ascii=False)
            if "error" in facts:
                print(f"ERROR: {facts['message']}")
            else:
                clasif = facts["clasificacion_automatica"]["categoria"]
                dur = facts["resumen_general"]["duracion_total_min"]
                print(f"OK ({dur} min, clasificado como {clasif}) -> {os.path.basename(out_path)}")
        except Exception as e:
            print(f"FALLO: {e}")

    print(f"\nListo. Los archivos .facts.json estan junto a cada .BIN.")
    print("Siguiente paso: pasar cada .facts.json a la IA usando el prompt de generate_report_prompt.txt")


if __name__ == "__main__":
    main()
