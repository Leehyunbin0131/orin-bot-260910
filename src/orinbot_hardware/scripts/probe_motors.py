#!/usr/bin/env python3
"""Protocol 2.0 PING/READ only. Never changes ID, mode, baud, torque or goals."""
import argparse
import json
import os
import sys

DEFAULT_PORT = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBIN9M3-if00-port0'


def read_registers(packet, port, motor_id):
    """Keep valid READ data when only the Protocol 2.0 hardware alert is set."""
    from dynamixel_sdk import COMM_SUCCESS
    item = {}
    registers = [('drive_mode', 10, 1), ('operating_mode', 11, 1),
                 ('torque_enabled', 64, 1), ('hardware_error', 70, 1),
                 ('bus_watchdog', 98, 1), ('goal_velocity_raw', 104, 4),
                 ('present_velocity_raw', 128, 4), ('position_raw', 132, 4),
                 ('voltage_raw', 144, 2), ('temperature_c', 146, 1)]
    for name, address, size in registers:
        value, comm, error = getattr(packet, f'read{size}ByteTxRx')(port, motor_id, address)
        if comm == COMM_SUCCESS and error & 0x80:
            item['hardware_alert'] = True
        # Bit 7 is a hardware alert; bits 0..6 indicate instruction failure.
        if comm != COMM_SUCCESS or error & 0x7f:
            item[name] = {'communication': packet.getTxRxResult(comm),
                          'device_error': packet.getRxPacketError(error)}
        else:
            if size == 4 and value >= 2**31:
                value -= 2**32
            item[name] = value
    status = item.get('hardware_error')
    if isinstance(status, int):
        item['hardware_error_flags'] = [name for bit, name in (
            (1, 'Input Voltage Error'), (4, 'Overheating Error'),
            (8, 'Motor Encoder Error'), (16, 'Electrical Shock Error'),
            (32, 'Overload Error')) if status & bit]
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baud', type=int, nargs='+', default=[57600])
    args = parser.parse_args()
    if not os.path.exists(args.port):
        parser.exit(2, f'장치가 없습니다: {args.port}\n')
    if not os.access(args.port, os.R_OK | os.W_OK):
        parser.exit(2, f'시리얼 접근 권한이 없습니다: {args.port}\n'
                    "이미 dialout에 등록된 계정이면 'newgrp dialout', 'jazzy' 순서로 실행한 뒤 다시 시도하세요.\n"
                    '아직 등록하지 않았다면 prepare_host.sh를 사용자 터미널에서 실행하세요.\n')
    try:
        from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
    except ImportError:
        parser.exit(2, 'dynamixel_sdk가 없습니다. ROS Jazzy 환경과 prepare_host.sh 설치를 확인하세요.\n')
    port = PortHandler(args.port)
    packet = PacketHandler(2.0)
    found = []
    try:
        if not port.openPort():
            raise RuntimeError(f'포트를 열 수 없습니다: {args.port}')
        for baud in args.baud:
            if not port.setBaudRate(baud):
                raise RuntimeError(f'지원하지 않는 baud: {baud}')
            devices, result = packet.broadcastPing(port)
            if result != COMM_SUCCESS:
                print(f'{baud}: {packet.getTxRxResult(result)}', file=sys.stderr)
                continue
            for motor_id, model_fw in sorted(devices.items()):
                item = {'baud_rate': baud, 'id': motor_id, 'model_number': model_fw[0],
                        'model': 'XM430-W210' if model_fw[0] == 1030 else 'unknown',
                        'firmware': model_fw[1]}
                # Only interpret the verified XM430-W210 register layout.
                if model_fw[0] == 1030:
                    item.update(read_registers(packet, port, motor_id))
                found.append(item)
    finally:
        port.closePort()
    print(json.dumps({'port': args.port, 'motors': found}, indent=2, ensure_ascii=False))
    if not found:
        print('모터 응답이 없습니다. 전원·통신선·현재 baud와 중복 ID를 확인하세요.', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(2)
