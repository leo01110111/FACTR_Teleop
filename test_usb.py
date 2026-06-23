from dynamixel_sdk import PortHandler, PacketHandler

DEV = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO91I-if00-port0"
BAUDS = [4000000, 57600, 115200, 1000000, 2000000, 3000000, 4500000]

for baud in BAUDS:
    ph = PortHandler(DEV)
    pk = PacketHandler(2.0)
    ph.openPort()
    ph.setBaudRate(baud)

    hits = []
    for sid in range(0, 31):
        model, res, err = pk.ping(ph, sid)
        if res == 0 and err == 0:
            hits.append((sid, model))

    ph.closePort()
    print(f"baud {baud}: {hits if hits else 'no response'}")