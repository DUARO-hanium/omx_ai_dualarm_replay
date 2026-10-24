#!/usr/bin/env python3
"""One-process supervisor for the bundled CSVs and ROBOTIS controllers."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from hardware import JOINTS, open_followers
from replay import DualFollowerReplayer


def inspect(take):
    metadata = json.loads((take / 'metadata.json').read_text())
    if metadata.get('format') != 'replay_final_csv_v1' or metadata.get('status') != 'complete':
        raise ValueError('Recording is incomplete or has an unknown format.')
    times = {}
    for side in ('left', 'right'):
        times[side] = []
        with (take / f'{side}.csv').open(newline='') as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != ['time', *JOINTS]:
                raise ValueError(f'{side}: unexpected CSV header')
            for row in reader:
                values = [float(row[key]) for key in ('time', *JOINTS)]
                if not all(math.isfinite(value) for value in values) or values[0] < 0:
                    raise ValueError(f'{side}: invalid time/position')
                if times[side] and not 0 < values[0] - times[side][-1] <= 0.5:
                    raise ValueError(f'{side}: invalid time sequence')
                times[side].append(values[0])
        if len(times[side]) < 2 or len(times[side]) != metadata['samples']:
            raise ValueError(f'{side}: sample count differs from metadata')
    if times['left'] != times['right']:
        raise ValueError('Left/right timestamps differ.')
    print(f'{take.name}: {metadata["samples"]} paired samples, '
          f'{times["left"][-1]-times["left"][0]:.3f}s; CSV angles unchanged.', flush=True)
    return metadata


def recording(name):
    take = (Path('/data') / name).resolve()
    if take.parent != Path('/data') or not take.is_dir():
        raise ValueError('Choose an existing recording folder directly under data/.')
    return take, inspect(take)


def check_session(metadata, args, session):
    if args.mock != session['mock']:
        raise ValueError('The active session and this command must use the same --mock setting.')
    for side in ('left', 'right'):
        port = getattr(args, side+'_port') or metadata['devices'][side]['device']
        if Path(port).resolve() != Path(session['ports'][side]).resolve():
            raise ValueError(f'{side}: use the USB port of the active session.')
        for mid in map(str, range(11, 17)):
            motor = metadata['devices'][side]['motors'][mid]
            active = session['devices'][side]['motors'][mid]
            if (any(motor[key] != active[key] for key in ('model', 'homing_offset'))
                    or (motor['drive_mode'] & 1) != (active['drive_mode'] & 1)):
                raise ValueError(f'{side} ID {mid}: coordinates differ from the active session.')


def release(ports):
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
    packet = PacketHandler(2.0)
    failures = []
    for side, path in ports.items():
        port = PortHandler(path)
        try:
            if not port.openPort() or not port.setBaudRate(1000000):
                raise RuntimeError(f'{side}: USB unavailable for torque OFF verification')
            for mid in range(11, 17):
                status, error = packet.write1ByteTxRx(port, mid, 64, 0)
                if status != COMM_SUCCESS or error & 0x7f:
                    failures.append(f'{side} ID {mid}: torque OFF write failed')
            for mid in range(11, 17):
                value, status, error = packet.read1ByteTxRx(port, mid, 64)
                if status != COMM_SUCCESS or error & 0x7f or value:
                    failures.append(f'{side} ID {mid}: torque OFF unconfirmed')
        except Exception as exc:
            failures.append(str(exc))
        finally:
            port.closePort()
    if failures:
        raise RuntimeError('; '.join(failures))
    print('All 12 motors: torque OFF confirmed.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('take', nargs='?', default='package_data')
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--align', type=float, default=10.0)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--inspect', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--torque-off-only', action='store_true')
    parser.add_argument('--confirmed', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--enqueue', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--request-id', default='', help=argparse.SUPPRESS)
    parser.add_argument('--left-port')
    parser.add_argument('--right-port')
    args = parser.parse_args()
    if not math.isfinite(args.align) or args.align <= .1 or not math.isfinite(args.speed) or args.speed <= 0:
        parser.error('Require --align > 0.1 and --speed > 0.')
    take, metadata = recording(args.take)
    args.take = take.name
    if args.inspect:
        return
    ports = {side: getattr(args, side+'_port') or metadata['devices'][side]['device']
             for side in ('left', 'right')}
    if args.torque_off_only:
        if args.mock:
            print('No active mock session; no USB access performed.')
            return
        from hardware import require_idle_controllers
        require_idle_controllers()
        release(ports)
        return
    if args.check:
        followers = {} if args.mock else open_followers(metadata, ports)
        for follower in followers.values():
            follower.close()
        print('Device coordinates, hardware errors and torque OFF checked; no settings written.')
        return
    if not args.mock and not args.confirmed:
        parser.error('Start real replay with ./replay.sh TAKE and press Enter.')

    status_path = Path('/results/status.json')
    request_path = Path('/results/next.json')
    if args.enqueue:
        current = json.loads(status_path.read_text())
        if current.get('protocol') != 2 or current['phase'] != 'holding':
            raise RuntimeError('The active session must have finished replay normally before the next replay.')
        check_session(metadata, args, current['session'])
        if request_path.exists():
            raise RuntimeError('Another replay request is already pending.')
        request = {key: getattr(args, key) for key in
                   ('take', 'align', 'speed', 'request_id', 'left_port', 'right_port')}
        temporary = request_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(request)+'\n')
        temporary.replace(request_path)
        print('Next replay requested; existing controllers and torque remain active.', flush=True)
        return

    request_path.unlink(missing_ok=True)
    outputs = []

    def new_output():
        path = Path('/results') / f'{args.take}-{datetime.now(timezone.utc):%Y%m%dT%H%M%S%fZ}'
        path.mkdir(parents=True)
        outputs.append(path)
        return path

    output = new_output()
    uid, gid = int(os.environ.get('HOST_UID', 0)), int(os.environ.get('HOST_GID', 0))
    session = {'mock': args.mock, 'ports': ports, 'devices': metadata['devices']}
    phase = 'starting'

    def state(next_phase, message):
        nonlocal phase
        phase = next_phase
        temporary = status_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps({'protocol': 2, 'phase': phase, 'message': message,
            'request_id': args.request_id, 'take': args.take, 'mock': args.mock,
            'output': str(output), 'session': session}, ensure_ascii=False)+'\n')
        os.chown(temporary, uid, gid)
        temporary.replace(status_path)

    def torque_off(_signal, _frame):
        nonlocal requested_off
        requested_off = True

    controller = None
    node = None
    activated = False
    requested_off = False
    log = (output/'controllers.log').open('w')

    def play():
        state('replaying', 'Approaching the first CSV pose, then replaying both arms.')
        try:
            node.run()
        finally:
            node.close_metrics()
        if not requested_off:
            state('holding', 'Replay finished; controllers retain the final CSV targets and accept the next replay.')
            print('REPLAY COMPLETE: holding final recorded pose. Start another replay or use ./torque_off.sh.', flush=True)

    def hold_error(exc):
        if phase != 'holding_error':
            print(f'Replay stopped: {exc}', file=sys.stderr, flush=True)
        if node and phase != 'holding_error':
            try:
                node.hold()
            except Exception as hold_exc:
                print(f'Hold could not be confirmed: {hold_exc}', file=sys.stderr)
            node.close_metrics()
        state('holding_error', str(exc))

    state('starting', 'Checking devices and starting the follower controllers.')
    signal.signal(signal.SIGUSR1, torque_off)
    signal.signal(signal.SIGTERM, torque_off)
    try:
        followers = {} if args.mock else open_followers(metadata, ports)
        try:
            for follower in followers.values():
                follower.prepare_hold()
            if followers:
                session['devices'] = {side: follower.identity for side, follower in followers.items()}
        finally:
            for follower in followers.values():
                follower.close()
        ros_args = ['--ros-args']
        for old, side in (('follower1', 'left'), ('follower2', 'right')):
            for topic in ('joint_states', 'dynamic_joint_states'):
                ros_args += ['-r', f'/{old}/{topic}:=/{side}/{topic}']
            ros_args += ['-r', f'/{old}/replay_controller/joint_trajectory:=/{side}/replay_controller/joint_trajectory']
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGUSR1, torque_off)
        signal.signal(signal.SIGTERM, torque_off)
        controller = subprocess.Popen([
            'ros2', 'launch', '/app/launch.py', f'mock:={str(args.mock).lower()}',
            f'left_port:={ports["left"]}', f'right_port:={ports["right"]}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        activated = True
        parameters = {'left_input_file': str(take/'left.csv'), 'right_input_file': str(take/'right.csv'),
                      'joint_prefixes': ['left_', 'right_'], 'start_delay_s': args.align,
                      'speed': args.speed, 'confirm': False, 'metrics_file': str(output/'metrics.csv')}
        node = DualFollowerReplayer(parameter_overrides=[Parameter(key, value=value) for key, value in parameters.items()])
        node.stop_requested = lambda: requested_off
        deadline = time.monotonic()+60
        while time.monotonic() < deadline and not requested_off:
            rclpy.spin_once(node, timeout_sec=.1)
            if controller.poll() is not None:
                raise RuntimeError(f'Controller launch stopped. See {output}/controllers.log')
            if node.ready():
                break
        if not requested_off:
            if not node.ready():
                raise RuntimeError(f'Controllers not ready. See {output}/controllers.log')
            play()
    except (KeyboardInterrupt, Exception) as exc:
        if not activated:
            state('error', str(exc))
            raise SystemExit(1)
        hold_error(exc)
    try:
        while activated and not requested_off:
            try:
                if controller.poll() is not None:
                    raise RuntimeError('Hardware controllers stopped; torque state is unconfirmed.')
                if node:
                    rclpy.spin_once(node, timeout_sec=.1)
                    if not node.ready():
                        raise RuntimeError('Follower feedback is unavailable while holding.')
                else:
                    time.sleep(.1)
                if request_path.exists():
                    request = json.loads(request_path.read_text())
                    request_path.unlink()
                    args.request_id = request['request_id']
                    if phase != 'holding':
                        raise RuntimeError('Cannot start another replay after a controller or replay error.')
                    next_take, next_metadata = recording(request['take'])
                    next_args = argparse.Namespace(**request, mock=args.mock)
                    check_session(next_metadata, next_args, session)
                    if requested_off:
                        continue
                    args.take = next_take.name
                    args.align, args.speed = request['align'], request['speed']
                    take, metadata = next_take, next_metadata
                    output = new_output()
                    node.set_recording([str(take/'left.csv'), str(take/'right.csv')],
                                       output/'metrics.csv', args.align, args.speed)
                    play()
            except (KeyboardInterrupt, Exception) as exc:
                hold_error(exc)
                time.sleep(.1)
    finally:
        signal.signal(signal.SIGUSR1, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if controller and controller.poll() is None:
            controller.send_signal(signal.SIGINT)
            try:
                controller.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(controller.pid, signal.SIGTERM)
                controller.wait(timeout=5)
        log.close()
        if requested_off:
            try:
                if args.mock:
                    print('Mock controllers stopped.', flush=True)
                else:
                    release(ports)
                state('off', 'Mock stopped.' if args.mock else 'All 12 motors: torque OFF verified.')
            except Exception as exc:
                state('error', f'Torque OFF not confirmed: {exc}')
                raise
        for directory in outputs:
            for path in (directory, *directory.rglob('*')):
                os.chown(path, uid, gid)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, EOFError) as exc:
        sys.exit(str(exc))
