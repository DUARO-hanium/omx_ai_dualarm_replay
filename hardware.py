"""OMX follower settings used by the hand-guided replay workspace."""
import fcntl
from pathlib import Path
import termios
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro

TICK_RAD = 0.0015339807878856412
JOINTS = ['joint1','joint2','joint3','joint4','joint5','gripper_joint_1']


def configure_description(urdf, side):
    for joint in urdf.findall('joint'):
        limit = joint.find('limit')
        if joint.get('type') == 'revolute' and limit is not None:
            limit.set('lower', str(angle(-1048575)))
            limit.set('upper', str(angle(1048575)))
    for interface in urdf.findall("ros2_control/joint/command_interface[@name='position']"):
        for param in list(interface):
            if param.get('name') in ('min', 'max'):
                interface.remove(param)
    for motor in urdf.findall('ros2_control/gpio'):
        mid = int(motor.find("param[@name='ID']").text)
        motor.find("param[@name='Operating Mode']").text = '5' if mid == 16 else '4'
        for name in ('Min Position Limit', 'Max Position Limit'):
            param = motor.find(f"param[@name='{name}']")
            if param is not None:
                motor.remove(param)
        interfaces = ['Hardware Error Status', 'Present Input Voltage']
        if mid >= 14:
            interfaces.append('Present Current')
        for name in interfaces:
            if motor.find(f"state_interface[@name='{name}']") is None:
                ET.SubElement(motor, 'state_interface', name=name)
        if mid == 16:
            for name, value in (('Current Limit', 100), ('Goal Current', 100), ('Shutdown', 53)):
                motor.find(f"param[@name='{name}']").text = str(value)


def follower_description(side, port='/dev/unused', mock=True):
    description = Path(get_package_share_directory('open_manipulator_description'))
    document = xacro.process_file(str(description / 'urdf/omx_f/omx_f.urdf.xacro'),
                                 mappings={'prefix': f'{side}_', 'use_sim': 'false',
                                           'use_mock_hardware': str(mock).lower(), 'port_name': port})
    urdf = ET.fromstring(document.toxml())
    configure_description(urdf, side)
    if mock:
        for interface in urdf.findall('ros2_control/gpio/state_interface'):
            ET.SubElement(interface, 'param', name='initial_value').text = '0'
    return urdf

def angle(raw):
    return float(raw) * TICK_RAD - 3.14159265359

def signed(value):
    return value if value < 2**31 else value - 2**32

class Follower:
    def __init__(self, side, path):
        from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
        self.ok = COMM_SUCCESS
        self.side = side
        self.held = False
        self.raw_positions = None
        self.port = PortHandler(path)
        self.packet = PacketHandler(2.0)
        try:
            if not self.port.openPort() or not self.port.setBaudRate(1000000):
                raise RuntimeError(f'{side}: cannot open {path} at 1 Mbps')
            import fcntl
            fcntl.ioctl(self.port.ser.fileno(), termios.TIOCEXCL)
            motors = {}
            for mid in range(11, 17):
                model, status, error = self.packet.ping(self.port, mid)
                if status != self.ok or error:
                    raise RuntimeError(f'{side} ID {mid}: ping failed ({status}, {error})')
                if model != (1060 if mid <= 13 else 1200):
                    raise RuntimeError(f'{side} ID {mid}: unsupported model {model}')
                motors[str(mid)] = {
                    'model': model, 'homing_offset': signed(self.read(mid, 20, 4)),
                    'drive_mode': self.read(mid, 10, 1), 'operating_mode': self.read(mid, 11, 1)}
            self.identity = {'device': path, 'motors': motors}
            self.reader = GroupSyncRead(self.port, self.packet, 64, 72)
            for mid in range(11, 17):
                if not self.reader.addParam(mid):
                    raise RuntimeError(f'{side}: cannot register ID {mid}')
            self.sample()
        except BaseException:
            self.close()
            raise

    def read(self, mid, address, size):
        method = {1: self.packet.read1ByteTxRx, 2: self.packet.read2ByteTxRx,
                  4: self.packet.read4ByteTxRx}[size]
        value, status, error = method(self.port, mid, address)
        if status != self.ok or error & 0x7F:
            raise RuntimeError(f'{self.side} ID {mid}: read {address} failed ({status}, {error})')
        return value

    def write(self, mid, address, size, value):
        method = {1: self.packet.write1ByteTxRx, 2: self.packet.write2ByteTxRx,
                  4: self.packet.write4ByteTxRx}[size]
        status, error = method(self.port, mid, address, value & ((1 << (8*size))-1))
        if status != self.ok or error & 0x7F:
            raise RuntimeError(f'{self.side} ID {mid}: write {address} failed ({status}, {error})')

    def prepare_hold(self):
        self.sample()
        urdf = follower_description(self.side)
        registers = {'Drive Mode': (10, 1), 'Operating Mode': (11, 1),
                     'Return Delay Time': (9, 1), 'Position P Gain': (84, 2),
                     'Position I Gain': (82, 2), 'Position D Gain': (80, 2),
                     'Profile Velocity': (112, 4), 'Profile Acceleration': (108, 4),
                     'Current Limit': (38, 2), 'Goal Current': (102, 2), 'Shutdown': (63, 1)}
        for gpio in urdf.findall('ros2_control/gpio'):
            mid = int(gpio.find("param[@name='ID']").text)
            for name, (address, size) in registers.items():
                param = gpio.find(f"param[@name='{name}']")
                if param is not None and self.read(mid, address, size) != int(param.text):
                    self.write(mid, address, size, int(param.text))
            identity = self.identity['motors'][str(mid)]
            identity.update(drive_mode=self.read(mid, 10, 1), operating_mode=self.read(mid, 11, 1))
        self.sample()

    def hold(self):
        if self.held:
            return
        positions = self.sample()
        targets = list(self.raw_positions)
        # All six current-position goals are acknowledged before any torque is enabled.
        for mid, raw in zip(range(11, 17), targets):
            self.write(mid, 116, 4, raw)
        self.held = True
        try:
            for mid in range(11, 17):
                self.write(mid, 64, 1, 1)
            actual = self.sample()
            if max(abs(a-b) for a, b in zip(actual, positions)) > 0.05:
                raise RuntimeError(f'{self.side}: position changed while enabling hold')
        except BaseException:
            self.release()
            raise

    def release(self):
        errors = []
        for mid in range(11, 17):
            try:
                self.write(mid, 64, 1, 0)
                if self.read(mid, 64, 1):
                    raise RuntimeError(f'{self.side} ID {mid}: torque OFF not confirmed')
            except (OSError, RuntimeError) as exc:
                errors.append(str(exc))
        self.held = bool(errors)
        if errors:
            raise RuntimeError('; '.join(errors))

    def sample(self):
        status = self.reader.txRxPacket()
        if status != self.ok:
            raise RuntimeError(f'{self.side}: {self.packet.getTxRxResult(status)}')
        positions, raw_positions = [], []
        for mid in range(11, 17):
            if not self.reader.isAvailable(mid, 64, 72):
                raise RuntimeError(f'{self.side} ID {mid}: incomplete feedback')
            if bool(self.reader.getData(mid, 64, 1)) != self.held:
                raise RuntimeError(f'{self.side} ID {mid}: unexpected torque state. Stop control and '
                                   'support the arm; torque will not be re-enabled automatically.')
            error = self.reader.getData(mid, 70, 1)
            if error:
                raise RuntimeError(f'{self.side} ID {mid}: hardware error 0x{error:02x}')
            raw = signed(self.reader.getData(mid, 132, 4))
            raw_positions.append(raw)
            positions.append(angle(raw))
        self.raw_positions = raw_positions
        return positions

    def close(self):
        if self.port.is_open:
            self.port.closePort()


def require_idle_controllers():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc / 'cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if any(arg.endswith(b'/controller_manager/ros2_control_node') for arg in args):
            raise RuntimeError('Stop existing ROS hardware controllers before replay.')


def open_followers(metadata, ports):
    require_idle_controllers()
    if any(not Path(port).is_char_device() for port in ports.values()):
        raise RuntimeError(f'Connect the recorded followers. Expected USB devices: {ports}')
    if len({Path(port).resolve() for port in ports.values()}) != 2:
        raise RuntimeError('Left and right must use different USB devices.')
    followers = {}
    try:
        for side, port in ports.items():
            follower = Follower(side, port)
            followers[side] = follower
            for mid, motor in follower.identity['motors'].items():
                previous = metadata['devices'][side]['motors'][mid]
                if (any(motor[key] != previous[key] for key in ('model', 'homing_offset'))
                        or (motor['drive_mode'] & 1) != (previous['drive_mode'] & 1)):
                    raise RuntimeError(f'{side} ID {mid}: motor coordinates differ from the recording.')
        return followers
    except BaseException:
        for follower in followers.values():
            follower.close()
        raise
