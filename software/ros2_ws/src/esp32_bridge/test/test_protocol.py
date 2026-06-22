import struct

from esp32_bridge.protocol import (
    MSG_BOARD_IDENTITY,
    MSG_ESTOP,
    MSG_ESTOP_CLEAR,
    ROLE_ARM,
    ROLE_CHASSIS,
    BoardIdentity,
    ChassisEstopMirror,
    FrameParser,
    RoleRouteTable,
    build_frame,
    parse_identity,
)


class FakeLink:
    pass


def test_frame_parser_resyncs_and_decodes_identity():
    identity_payload = struct.pack('<BBH', ROLE_CHASSIS, 1, 0x001D)
    frame = build_frame(MSG_BOARD_IDENTITY, identity_payload)
    corrupt = bytearray(build_frame(MSG_BOARD_IDENTITY, b'bad'))
    corrupt[-1] ^= 0xFF

    parser = FrameParser()
    parsed = parser.feed(b'noise' + bytes(corrupt) + b'\xAA') + parser.feed(frame)

    assert parsed == [(MSG_BOARD_IDENTITY, identity_payload)]
    identity = parse_identity(parsed[0][1])
    assert identity.role == ROLE_CHASSIS
    assert identity.protocol_version == 1
    assert identity.capabilities == 0x001D


def test_role_route_table_assigns_and_replaces_roles():
    routes = RoleRouteTable()
    chassis_a = FakeLink()
    chassis_b = FakeLink()
    arm = FakeLink()

    assert routes.assign(chassis_a, BoardIdentity(ROLE_CHASSIS, 1, 0)) is None
    assert routes.get(ROLE_CHASSIS) is chassis_a
    assert routes.assign(arm, BoardIdentity(ROLE_ARM, 1, 0)) is None
    assert routes.get(ROLE_ARM) is arm
    assert routes.assign(chassis_b, BoardIdentity(ROLE_CHASSIS, 1, 0)) is chassis_a
    assert routes.get(ROLE_CHASSIS) is chassis_b

    routes.clear_link(chassis_b)
    assert routes.get(ROLE_CHASSIS) is None
    assert routes.get(ROLE_ARM) is arm


def test_chassis_estop_mirror_emits_only_on_transitions():
    mirror = ChassisEstopMirror()
    parser = FrameParser()

    assert mirror.update(False) is None

    stop = mirror.update(True)
    assert parser.feed(stop) == [(MSG_ESTOP, b'')]
    assert mirror.update(True) is None

    clear = mirror.update(False)
    assert parser.feed(clear) == [(MSG_ESTOP_CLEAR, b'')]
    assert mirror.update(False) is None
