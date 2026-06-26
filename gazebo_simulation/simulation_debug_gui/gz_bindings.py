"""
Load the Gazebo Transport Python bindings without hard-coding a version.

The official bindings are versioned to your Gazebo release:
    Harmonic  -> gz.transport13 / gz.msgs10
    Ionic     -> gz.transport14 / gz.msgs11
    (older)   -> gz.transport12 / gz.msgs9, etc.

These come from your *system* Gazebo install (apt: gz-transport-python /
python3-gz-transportXX). They are NOT pip-installable in general, so this module
fails gracefully: if the bindings aren't present the rest of the app still runs,
and the camera/lidar panels simply show "waiting for Gazebo".

Usage:
    gz = load_gz()                # raises GzUnavailable if not installed
    node = gz.Node()
    img_cls = gz.msg("image", "Image")
    node.subscribe(img_cls, "/topic", cb)
"""
from __future__ import annotations

import importlib

import config


class GzUnavailable(RuntimeError):
    pass


# Known (transport_version, msgs_version) pairs, newest first.
_KNOWN = [(15, 12), (14, 11), (13, 10), (12, 9), (11, 8)]


class _Gz:
    def __init__(self, transport_mod, msgs_version: int):
        self.Node = transport_mod.Node
        self._msgs_version = msgs_version

    def msg(self, module: str, classname: str):
        """e.g. msg('image', 'Image') -> gz.msgsXX.image_pb2.Image"""
        mod = importlib.import_module(f"gz.msgs{self._msgs_version}.{module}_pb2")
        return getattr(mod, classname)


def _try(transport_v: int, msgs_v: int) -> _Gz | None:
    try:
        tmod = importlib.import_module(f"gz.transport{transport_v}")
        importlib.import_module(f"gz.msgs{msgs_v}.image_pb2")  # sanity check
        return _Gz(tmod, msgs_v)
    except Exception:
        return None


def load_gz() -> _Gz:
    if config.GZ_TRANSPORT_VERSION is not None:
        tv = config.GZ_TRANSPORT_VERSION
        # pair the requested transport version with its known msgs version
        mv = next((m for (t, m) in _KNOWN if t == tv), tv - 3)
        gz = _try(tv, mv)
        if gz:
            return gz
        raise GzUnavailable(
            f"gz.transport{tv}/gz.msgs{mv} not importable. Check your Gazebo "
            f"Python bindings install.")
    for tv, mv in _KNOWN:
        gz = _try(tv, mv)
        if gz:
            return gz
    raise GzUnavailable(
        "No Gazebo Python bindings found. Install them from your Gazebo distro "
        "(e.g. `apt install python3-gz-transport13`) or set "
        "GZ_TRANSPORT_VERSION in config.py.")
