# Herramienta de Análisis de Logs ArduPilot — Uso Local en el Laboratorio

## Qué es esto

Dos archivos que reemplazan lo que yo (Claude) he estado haciendo manualmente:

1. **`extract_facts.py`** — Script Python. Lee un `.BIN` y calcula TODOS los
   números relevantes (GPS, vibración, actitud, energía, errores, etc.) usando
   fórmulas fijas y umbrales conocidos. No usa IA. No interpreta nada. Solo
   produce un archivo `.facts.json` con hechos verificables.

2. **`generate_report_prompt.txt`** — El prompt que se le pega a GPT/Gemini
   JUNTO con el `.facts.json` para que redacte el informe final en Markdown.
   El prompt le prohíbe explícitamente inventar cualquier número que no esté
   en el JSON.

## Por qué esta separación es importante

El problema original con Antigravity era que el agente calculaba Y redactaba
al mismo tiempo, lo que le permitía "rellenar" con suposiciones cuando algo
no tenía sentido. Aquí separamos las dos cosas:

```
  .BIN  →  [extract_facts.py]  →  .facts.json  →  [GPT/Gemini + prompt]  →  Informe.md
            (matemática pura,        (hechos          (solo redacta y
             sin IA, 100%             verificables)     compara contra
             reproducible)                              umbrales del JSON)
```

Si el informe final dice algo raro, el error solo puede estar en una de dos
partes claramente separadas: el cálculo (revisable y reproducible) o la
redacción (se corrige ajustando el prompt, no el código).

## Instalación (una sola vez por computadora)

```bash
pip install pymavlink
```

## Uso día a día

### Paso 1 — Extraer los hechos de uno o varios logs

```bash
# Un solo log
python extract_facts.py 00000036.BIN

# Varios logs a la vez
python extract_facts.py 00000007.BIN 00000025.BIN 00000036.BIN

# Toda una carpeta
python extract_facts.py /ruta/a/carpeta_con_bins/
```

Esto genera un `NOMBRE.facts.json` al lado de cada `.BIN`. Tarda segundos por
archivo, no minutos — es matemática simple, no hay llamadas a ningún servicio.

### Paso 2 — Generar el informe con GPT/Gemini

1. Abre `generate_report_prompt.txt`.
2. Pega el prompt completo en GPT o Gemini.
3. Justo después del prompt, pega el contenido del `.facts.json`
   correspondiente (o adjúntalo como archivo si la interfaz lo permite).
4. La IA devuelve el informe en Markdown, listo para copiar a Word o PDF.

Repite el paso 2 por cada log que quieras documentar. El paso 1 ya lo hiciste
una vez para todos los logs.

## Ajustar los umbrales y la especificación de batería

Al inicio de `extract_facts.py` hay dos diccionarios editables:

```python
THRESHOLDS = {
    "gps_hdop_excelente": 1.5,
    "vibe_critico": 30.0,
    "att_roll_critico_fw": 70.0,
    ...
}

BATTERY_SPEC = {
    "celdas": None,        # <- poner aquí el número real (ej: 10)
    "capacidad_mah": None, # <- poner aquí la capacidad real (ej: 14400)
}
```

**Importante:** mientras `BATTERY_SPEC["celdas"]` sea `None`, el script NUNCA
asumirá cuántas celdas tiene la batería. Lo dirá explícitamente como "no
verificado" en el JSON, y el prompt le indica a la IA que haga lo mismo en el
informe. Edítenlo en cuanto confirmen físicamente la configuración real.

## Qué hace el script que antes hacía yo manualmente

- Usa **BARO** para altitud, no GPS (evita el problema de altitud en cero
  cuando el GPS pierde fix).
- Usa **percentil 95** para vibración, no el máximo puntual (evita falsas
  alarmas por picos momentáneos).
- Detecta el fin del vuelo activo usando mensajes de **ATT/CTUN/GPS/CURR/TECS**,
  nunca usando IMU (que se sigue grabando en tierra sin parar).
- Busca eventos críticos como **RC failsafe** y **EKF stopped aiding** como
  texto libre en mensajes MSG, no solo en el campo ERR estructurado (muchos
  fallos importantes de ArduPilot NO aparecen como ERR).
- Solo marca un error/gap/failsafe como sospechoso si el dron estaba
  **efectivamente en el aire** en ese instante exacto — un aterrizaje normal
  seguido de espera en tierra no se confunde con un crash.
- Si los datos de corriente son sospechosamente bajos (<5A durante vuelo),
  lo marca como posible sensor sin calibrar, no como bajo consumo real.

## Validación

Este script fue probado contra los 3 logs que ya analizamos manualmente en
la conversación con Claude (00000007, 00000025, 00000036) y los números
coinciden exactamente con los informes ya entregados:

| Log | Clasificación | GPS disponible | Vibración p95 (X/Y/Z) |
|---|---|---|---|
| 00000007 | Prueba en tierra | 100% | 0.17 / 0.25 / 0.41 |
| 00000025 | Vuelo normal/misión | 100% | 8.08 / 13.66 / 12.31 |
| 00000036 | Posible accidente | 54.7% | (no aplica - ver eventos graves) |

## Limitaciones conocidas

- El script NO interpreta causa raíz — eso lo hace la IA en el paso 2, y solo
  con los datos que el JSON le da.
- Si el log no tiene mensajes CURR (batería), el script lo indica como "no
  disponible" — no inventa valores.
- El umbral de "vuelo real" (5m de altitud sostenida) puede necesitar ajuste
  si el dron del labo opera con cambios de altitud más pequeños en pruebas
  intencionales de baja altura.
