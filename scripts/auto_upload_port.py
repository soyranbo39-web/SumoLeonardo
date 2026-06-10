Import("env")

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None


def detectar_puerto_leonardo():
    if list_ports is None:
        return None

    candidatos = []
    for puerto in list_ports.comports():
        descripcion = f"{puerto.description} {puerto.hwid}"
        if "Leonardo" in descripcion or "2341:8036" in descripcion or "2341:0036" in descripcion:
            candidatos.append(puerto.device)

    if candidatos:
        return candidatos[0]

    return None


puerto = detectar_puerto_leonardo()
if puerto:
    print(f"Puerto de subida detectado automaticamente: {puerto}")
    env.Replace(UPLOAD_PORT=puerto)